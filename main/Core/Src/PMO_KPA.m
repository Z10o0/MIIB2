%% ============================================================
%  ICM-45686 | STM32H723 | UART приёмник + управление режимами
%
%  Команды → прибор:
%    'R' (0x52) → MODE_RAW   — raw пакеты 0x55AA (20 байт)
%    'C' (0x43) → MODE_CALIB — calib пакеты 0x55BB (34 байта)
%    'S' (0x53) → MODE_IDLE  — стоп
%
%  Формат RAW-пакета (20 байт, little-endian):
%    [0xAA 0x55] [uint32 sample_idx] [6 × int16: ax ay az gx gy gz] [int8 temp] [uint8 pad]
%
%  Формат CALIB-пакета (34 байта, little-endian):
%    [0xBB 0x55] [uint32 sample_idx] [7 × float32: ax ay az gx gy gz temp]
%%
%%  Тактирование UART:
%%    PLL2Q = 48 МГц → BaudRate = 3 000 000 бод (погрешность 0%)
%%    Нагрузка при 6400 Гц CALIB = 72.5%
%% ============================================================
clear; close all; clc;

%% ── НАСТРОЙКИ ────────────────────────────────────────────────
PORT        = 'COM10';
BAUD        = 3000000;   % изменено с 921600 → 3 000 000 бод
N_SAMPLES   = 64000;     % 10 секунд при 6400 Гц
ODR_SET_HZ  = 6400;      % изменено с 1600 → 6400 Гц

% Режим работы прибора: 'RAW' или 'CALIB'
DEVICE_MODE = 'CALIB';

% Масштабные коэффициенты (используются только в режиме RAW)
ACC_FSR_G   = 2.0;
GYR_FSR_DPS = 15.625;
ACC_SCALE   = ACC_FSR_G   * 9.80665 / 32768.0;
GYR_SCALE   = GYR_FSR_DPS            / 32768.0;
TEMP_SCALE  = 0.5;    % T[°C] = raw * 0.5 + 25
TEMP_OFFSET = 25.0;

OUTPUT_CSV = sprintf('icm45686_%s_%s.csv', lower(DEVICE_MODE), ...
                     datestr(now, 'yyyymmdd_HHMMSS'));

%% ── Байты команд ─────────────────────────────────────────────
CMD_RAW   = uint8('R');   % 0x52
CMD_CALIB = uint8('C');   % 0x43
CMD_STOP  = uint8('S');   % 0x53

%% ── Параметры пакета (зависят от режима) ─────────────────────
switch upper(DEVICE_MODE)
    case 'RAW'
        SYNC_B1      = uint8(0xAA);
        SYNC_B2      = uint8(0x55);
        CMD_SEND     = CMD_RAW;
        PACKET_BYTES = 20;
    case 'CALIB'
        SYNC_B1      = uint8(0xBB);
        SYNC_B2      = uint8(0x55);
        CMD_SEND     = CMD_CALIB;
        PACKET_BYTES = 34;
    otherwise
        error('DEVICE_MODE должен быть "RAW" или "CALIB"');
end

%% ── Открываем порт ───────────────────────────────────────────
s         = serialport(PORT, BAUD);
s.Timeout = 10;
flush(s);

%% ── Отправляем команду режима ────────────────────────────────
write(s, CMD_SEND, 'uint8');
fprintf('Отправлена команда: %s (0x%02X)\n', DEVICE_MODE, CMD_SEND);

pause(0.2);   % даём прибору время переключить режим
flush(s);     % сбрасываем байты, пришедшие до смены режима

fprintf('Порт %s открыт @ %d бод. Режим: %s. Ожидаю %d отсчётов...\n', ...
        PORT, BAUD, DEVICE_MODE, N_SAMPLES);

%% ── Преаллокация ─────────────────────────────────────────────
idx_raw   = zeros(N_SAMPLES, 1, 'uint32');
imu_data  = zeros(N_SAMPLES, 7, 'double');

drop_cnt = 0;
received = 0;
ring     = uint8([]);
tic;

%% ── Приём ────────────────────────────────────────────────────
while received < N_SAMPLES
    avail = s.NumBytesAvailable;
    if avail > 0
        ring = [ring, read(s, avail, 'uint8')]; %#ok<AGROW>
    else
        pause(0.0002);   % уменьшен с 0.5 мс до 0.2 мс для 6400 Гц
        continue
    end

    while numel(ring) >= PACKET_BYTES
        % Ищем синхрослово
        idx_sync = find(ring(1:end-1) == SYNC_B1 & ...
                        ring(2:end)   == SYNC_B2, 1);
        if isempty(idx_sync)
            ring = ring(end);
            break
        end
        if idx_sync > 1
            ring = ring(idx_sync:end);
        end
        if numel(ring) < PACKET_BYTES
            break
        end

        pkt  = ring(1:PACKET_BYTES);
        ring = ring(PACKET_BYTES+1:end);

        % Парсинг заголовка: байты 1-2 = sync, байты 3-6 = uint32 idx
        s_idx = typecast(pkt(3:6), 'uint32');

        if strcmpi(DEVICE_MODE, 'RAW')
            raw16 = double(typecast(pkt(7:18), 'int16'));   % 6 значений
            temp_raw = double(typecast(pkt(19), 'int8'));
            imu_data(received+1, :) = [raw16(:)', temp_raw];
        else % CALIB
            imu_data(received+1, :) = double(typecast(pkt(7:34), 'single'));
        end

        received          = received + 1;
        idx_raw(received) = s_idx;

        % Детектор пропусков
        if received > 1
            if s_idx ~= idx_raw(received-1) + 1
                drop_cnt = drop_cnt + 1;
            end
        end

        if mod(received, ODR_SET_HZ) == 0
            fprintf('  Принято: %d / %d  (пропусков: %d)\n', ...
                received, N_SAMPLES, drop_cnt);
        end

        if received >= N_SAMPLES
            break
        end
    end
end

elapsed   = toc;
real_rate = N_SAMPLES / elapsed;

%% ── Останавливаем прибор ─────────────────────────────────────
write(s, CMD_STOP, 'uint8');
fprintf('Отправлена команда STOP.\n');
pause(0.05);
clear s;

%% ── Итог ─────────────────────────────────────────────────────
fprintf('\n── Итог приёма ──────────────────────────────────\n');
fprintf('  Режим:             %s\n',     DEVICE_MODE);
fprintf('  UART скорость:     %d бод\n', BAUD);
fprintf('  Принято:           %d отсчётов за %.3f с\n', N_SAMPLES, elapsed);
fprintf('  Реальная частота:  %.1f Гц\n',  real_rate);
fprintf('  Настроенная ODR:   %d Гц\n',    ODR_SET_HZ);
fprintf('  Пропущено:         %d (%.3f%%)\n', ...
    drop_cnt, 100.0 * drop_cnt / N_SAMPLES);
fprintf('─────────────────────────────────────────────────\n\n');

%% ── Конвертация в физические единицы ────────────────────────
if strcmpi(DEVICE_MODE, 'CALIB')
    acc_x = imu_data(:,1);
    acc_y = imu_data(:,2);
    acc_z = imu_data(:,3);
    gyr_x = imu_data(:,4);
    gyr_y = imu_data(:,5);
    gyr_z = imu_data(:,6);
    temp  = imu_data(:,7);
    raw_cols = zeros(N_SAMPLES, 7);
else
    acc_x = imu_data(:,1) * ACC_SCALE;
    acc_y = imu_data(:,2) * ACC_SCALE;
    acc_z = imu_data(:,3) * ACC_SCALE;
    gyr_x = imu_data(:,4) * GYR_SCALE;
    gyr_y = imu_data(:,5) * GYR_SCALE;
    gyr_z = imu_data(:,6) * GYR_SCALE;
    temp  = imu_data(:,7) * TEMP_SCALE + TEMP_OFFSET;
    raw_cols = imu_data;
end

t_real = (0 : N_SAMPLES-1)' / real_rate;
t_nom  = (0 : N_SAMPLES-1)' / ODR_SET_HZ;

%% ── CSV ──────────────────────────────────────────────────────
T = table( ...
    t_real, t_nom, double(idx_raw), ...
    raw_cols(:,1), raw_cols(:,2), raw_cols(:,3), ...
    raw_cols(:,4), raw_cols(:,5), raw_cols(:,6), raw_cols(:,7), ...
    acc_x, acc_y, acc_z, ...
    gyr_x, gyr_y, gyr_z, ...
    temp, ...
    'VariableNames', { ...
        't_real_s','t_nom_s','sample_idx', ...
        'acc_x_raw','acc_y_raw','acc_z_raw', ...
        'gyr_x_raw','gyr_y_raw','gyr_z_raw','temp_raw', ...
        'acc_x_ms2','acc_y_ms2','acc_z_ms2', ...
        'gyr_x_dps','gyr_y_dps','gyr_z_dps','temp_c'});

writetable(T, OUTPUT_CSV);
fprintf('CSV сохранён: %s\n', OUTPUT_CSV);

%% ── Графики ──────────────────────────────────────────────────
figure('Name', sprintf('ICM-45686 | %s @ %d бод | %d Гц', DEVICE_MODE, BAUD, ODR_SET_HZ), ...
       'Color', [0.1 0.1 0.1]);
set(gcf, 'Units','normalized','Position',[0.05 0.05 0.90 0.85]);

ax1 = subplot(3,1,1);
plot(t_real, acc_x,'b', t_real, acc_y,'r', t_real, acc_z,'y');
legend('X','Y','Z','Location','northeast','TextColor','w');
ylabel('м/с²','Color','w');
title(sprintf('Акселерометр  [%s]  ODR=%d Гц', DEVICE_MODE, ODR_SET_HZ),'Color','w');
set(ax1,'Color',[0.13 0.13 0.13],'XColor','w','YColor','w'); grid on;

ax2 = subplot(3,1,2);
plot(t_real, gyr_x,'b', t_real, gyr_y,'r', t_real, gyr_z,'y');
legend('X','Y','Z','Location','northeast','TextColor','w');
ylabel('°/с','Color','w'); title('Гироскоп','Color','w');
set(ax2,'Color',[0.13 0.13 0.13],'XColor','w','YColor','w'); grid on;

ax3 = subplot(3,1,3);
plot(t_real, temp,'b');
ylabel('°C','Color','w'); xlabel('t, с','Color','w');
title(sprintf('Температура  |  %.1f Гц  |  Пропусков: %d  |  UART: %d бод', ...
    real_rate, drop_cnt, BAUD),'Color','w');
set(ax3,'Color',[0.13 0.13 0.13],'XColor','w','YColor','w'); grid on;

linkaxes([ax1 ax2 ax3],'x');

%% ── Осреднение по окнам ──────────────────────────────────────
WIN_SEC  = 60.0;
WIN_SAMP = round(WIN_SEC * real_rate);
n_win    = floor(N_SAMPLES / WIN_SAMP);

if n_win == 0
    fprintf('Данных меньше %.0f с — осреднение пропущено.\n', WIN_SEC);
else
    gx_avg = zeros(n_win,1);
    az_avg = zeros(n_win,1);
    tc_avg = zeros(n_win,1);
    tw     = zeros(n_win,1);

    for k = 1:n_win
        i1 = (k-1)*WIN_SAMP + 1;
        i2 =  k   *WIN_SAMP;
        gx_avg(k) = mean(gyr_x(i1:i2));
        az_avg(k) = mean(acc_z(i1:i2));
        tc_avg(k) = mean(temp (i1:i2));
        tw(k)     = t_real(i1) + WIN_SEC/2;
    end

    fprintf('\n── Осреднение по %.1f с (%d окон) ────────────────\n', WIN_SEC, n_win);
    fprintf('  %-5s  %-10s  %-14s  %-10s\n','Окно','t_центр,с','gyr_x_dps','acc_z_ms2');
    for k = 1:n_win
        fprintf('  %-5d  %-10.3f  %+.6f     %+.6f\n', ...
            k, tw(k), gx_avg(k), az_avg(k));
    end
end

%% ── Вариация Аллана (раскомментировать при необходимости) ────
%
% gyr_x_dph = gyr_x * 3600;   % перевод °/с → °/ч для ADEV
%
% [avar, tau] = allanvar(gyr_x_dph, 'octave', real_rate);
% adev = sqrt(avar);
%
% figure;
% loglog(tau, adev);
% xlabel('\tau, с'); ylabel('\sigma(\tau), °/ч');
% title('Allan Deviation — gyr\_x'); grid on;
%
% % Температурная компенсация дрейфа
% p = polyfit(temp, gyr_x_dph, 4);
% temp_drift_model = polyval(p, temp);
% gyr_x_clean = gyr_x_dph - temp_drift_model;
%
% [avar_c, tau_c] = allanvar(gyr_x_clean, 'octave', real_rate);
% adev_c = sqrt(avar_c);
%
% figure;
% loglog(tau, adev, 'r', tau_c, adev_c, 'g');
% legend('Исходный','После темп. компенсации');
% xlabel('\tau, с'); ylabel('\sigma(\tau), °/ч');
% title('Allan Deviation — до/после температурной компенсации'); grid on;
