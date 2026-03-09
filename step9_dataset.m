% =========================================================================
% ADIM 9: ZENGİN DAE EĞİTİM VERİSİ OLUŞTURMA
% DAB SoOP TDOA LocalizationNO
% =========================================================================
% Seçilen parametreler:
%   - Farklı irtifa: 50, 100, 200, 500m
%   - Farklı UAV hızı: hover(0), yavaş(10), hızlı(30) m/s
%   - Ortam: Kentsel (yoğun NLOS) + Karma (geçiş)
%   - Teknik: Farklı multipath yoğunluğu + UAV motor gürültüsü
%
% Hedef: ~10.000+ korelasyon profili çifti (kirli + temiz)
% Çıktı: 'dae_training_data.mat'
% =========================================================================

clear all;
close all;
clc;

fprintf('=============================================\n');
fprintf('  ADIM 9: DAE Eğitim Verisi Oluşturuluyor\n');
fprintf('=============================================\n\n');

load('geometry_data.mat');
load('dab_signal_data.mat');

% =========================================================================
% BÖLÜM 1: SENARYO PARAMETRELERİ
% =========================================================================

% --- İrtifa Senaryoları ---
altitude_scenarios = [50, 100, 200, 500];   % [metre]
N_alt = length(altitude_scenarios);

% --- UAV Hız Senaryoları ---
% hover = askıda durma (0 m/s) → Doppler yok
% yavaş = 10 m/s (~36 km/h)
% hızlı = 30 m/s (~108 km/h)
speed_scenarios  = [0, 10, 30];             % [m/s]
speed_labels     = {'Hover', 'Yavaş', 'Hızlı'};
N_speed = length(speed_scenarios);

% --- Ortam Senaryoları ---
% Kentsel: çok bina → yoğun NLOS, güçlü multipath
% Karma:   geçiş bölgesi → orta NLOS, orta multipath
env_scenarios = struct();

% Kentsel
env_scenarios(1).name         = 'Kentsel';
env_scenarios(1).nlos_prob    = 0.6;        % %60 NLOS olasılığı
env_scenarios(1).nlos_delay   = [100, 800]; % ns
env_scenarios(1).nlos_loss    = [5, 25];    % dB
env_scenarios(1).shadow_std   = 10;         % dB (çok dalgalı)
env_scenarios(1).n_paths      = 5;          % Çok yansıma
env_scenarios(1).path_delays  = [0, 0.3, 0.7, 1.2, 2.0];    % µs
env_scenarios(1).path_powers  = [0, -6, -10, -15, -20];      % dB

% Karma
env_scenarios(2).name         = 'Karma';
env_scenarios(2).nlos_prob    = 0.3;        % %30 NLOS
env_scenarios(2).nlos_delay   = [50, 400];
env_scenarios(2).nlos_loss    = [8, 20];
env_scenarios(2).shadow_std   = 6;
env_scenarios(2).n_paths      = 3;
env_scenarios(2).path_delays  = [0, 0.5, 1.2];
env_scenarios(2).path_powers  = [0, -8, -15];

N_env = length(env_scenarios);

% --- SNR Aralığı ---
SNR_values = [0, 5, 10, 15, 20];           % dB
N_snr = length(SNR_values);

% --- UAV Motor Gürültüsü ---
% UAV motorları ve pervaneleri RF gürültüsü üretiyor
% Genellikle band içi bozucu olarak modellenir
% Darbe gürültüsü: ara sıra güçlü, kısa süreli bozucu
motor_noise_power_dB = -20;   % SNR'ye göre motor gürültüsü gücü
motor_pulse_prob     = 0.05;  % Her örnekte darbe olasılığı

% Taşıyıcı frekans (Doppler için)
f_carrier = 194e6;            % DAB Band-III [Hz]

fprintf('Senaryo Matrisi:\n');
fprintf('  İrtifa      : %s m\n', num2str(altitude_scenarios));
fprintf('  UAV hızı    : %s m/s\n', num2str(speed_scenarios));
fprintf('  Ortam       : Kentsel, Karma\n');
fprintf('  SNR         : %s dB\n', num2str(SNR_values));
fprintf('  Toplam kombo: %d × %d × %d × %d = %d\n\n', ...
        N_alt, N_speed, N_env, N_snr, N_alt*N_speed*N_env*N_snr);

% =========================================================================
% BÖLÜM 2: YARDIMCI FONKSİYONLAR
% =========================================================================

    % AWGN
    function s = add_awgn(x, snr_dB)
        p = mean(abs(x).^2);
        s = x + sqrt(p/10^(snr_dB/10)) * randn(size(x));
    end

    % Multipath
    function y = add_multipath(x, delay_us, power_dB, Fs)
        delays  = round(delay_us * 1e-6 * Fs);
        powers  = 10.^(power_dB / 10);
        y = zeros(size(x));
        for p = 1:length(delays)
            d = delays(p);
            if d == 0
                y = y + sqrt(powers(p)) * x;
            elseif d < length(x)
                y = y + sqrt(powers(p)) * [zeros(1,d), x(1:end-d)];
            end
        end
    end

    % Doppler
    function y = add_doppler(x, doppler_hz, Fs)
        if doppler_hz == 0; y = x; return; end
        t = (0:length(x)-1) / Fs;
        y = real(x .* real(exp(1j * 2*pi * doppler_hz * t)));
    end

    % NLOS
    function [y, is_nlos] = add_nlos(x, prob, delay_range_ns, loss_range_dB, Fs)
        is_nlos = rand() < prob;
        if ~is_nlos; y = x; return; end
        delay_ns  = delay_range_ns(1) + rand()*diff(delay_range_ns);
        loss_dB   = loss_range_dB(1)  + rand()*diff(loss_range_dB);
        d_samp    = round(delay_ns * 1e-9 * Fs);
        att       = 10^(-loss_dB/20);
        if d_samp > 0 && d_samp < length(x)
            y = att * [zeros(1,d_samp), x(1:end-d_samp)];
        else
            y = att * x;
        end
    end

    % Log-normal gölgeleme
    function y = add_shadowing(x, sigma_dB)
        gain = 10^(sigma_dB * randn() / 20);
        y    = x * gain;
    end

    % UAV motor gürültüsü
    % Motor ve pervane elektriksel gürültü üretiyor:
    %   1. Sürekli düşük seviye broadband gürültü
    %   2. Ara sıra güçlü darbeler (commutator switching)
    function y = add_motor_noise(x, noise_power_dB, pulse_prob)
        % Sürekli bileşen
        sig_power    = mean(abs(x).^2);
        noise_power  = sig_power * 10^(noise_power_dB/10);
        cont_noise   = sqrt(noise_power) * randn(size(x));

        % Darbe bileşeni (impulsive noise)
        pulse_mask   = rand(size(x)) < pulse_prob;
        pulse_amp    = sqrt(noise_power) * 10;   % Darbeler çok güçlü
        impulse_noise= pulse_mask .* (pulse_amp * randn(size(x)));

        y = x + cont_noise + impulse_noise;
    end

    % Saat kayması
    function y = add_clock(x, offset_ns, drift_ppb, Fs)
        oi = round(offset_ns * 1e-9 * Fs);
        if oi > 0
            y = [zeros(1,oi), x(1:end-oi)];
        elseif oi < 0
            y = [x(-oi+1:end), zeros(1,-oi)];
        else
            y = x;
        end
        t = (0:length(y)-1) / Fs;
        y = real(y .* real(exp(1j*2*pi*drift_ppb*1e-9*Fs*t)));
    end

    % Kuantizasyon
    function y = add_quant(x, bits)
        m = max(abs(x));
        if m == 0; y = x; return; end
        s = 2/2^bits;
        y = round((x/m)/s)*s*m;
    end

    % GCC-PHAT TOA kestirimi
    function [toa, prof] = gcc_phat(rx, ref, n_null, n_prs)
        s   = n_null+1;
        e   = min(s+n_prs-1, length(rx));
        seg = rx(s:e);
        ml  = min(length(seg), length(ref));
        seg = seg(1:ml); ref = ref(1:ml);
        N2  = 2^nextpow2(2*ml-1);
        G   = fft(seg,N2).*conj(fft(ref,N2));
        G   = G./(abs(G)+1e-10);
        cr  = real(ifft(G,N2));
        prof= abs(cr)/(max(abs(cr))+1e-10);
        [~,pk] = max(prof(1:floor(N2/2)));
        toa = n_null+pk-1;
    end

    % Temiz (ideal) korelasyon profili üret
    function prof = make_clean_profile(true_delay_samples, n_prs, corr_len)
        % Gürültüsüz PRS kendisiyle korelasyon → delta fonksiyonu
        % Sonra doğru gecikmeye kaydır
        prof = zeros(1, corr_len);
        peak_pos = round(mod(true_delay_samples, corr_len)) + 1;
        if peak_pos >= 1 && peak_pos <= corr_len
            % Gaussian şekilli tepe (gerçekçi)
            sigma = 2;   % Tepe genişliği [örnek]
            for k = max(1, peak_pos-10):min(corr_len, peak_pos+10)
                prof(k) = exp(-(k-peak_pos)^2 / (2*sigma^2));
            end
        end
        prof = prof / (max(prof) + 1e-10);
    end

% =========================================================================
% BÖLÜM 3: VERİ TOPLAMA DÖNGÜSÜ
% =========================================================================

corr_len = 2*N_prs - 1;

% Ön tahmin: toplam örnek sayısı
N_total_est = N_alt * N_speed * N_env * N_snr * N_pos * 4;
fprintf('Tahmini toplam örnek: %d\n\n', N_total_est);

% Dinamik büyüyen hücreler kullan
all_dirty   = {};   % Kirli korelasyon profilleri
all_clean   = {};   % Temiz (hedef) profiller
all_labels  = {};   % Metadata (senaryo bilgisi)
all_toa_err = [];   % TOA hataları [m]

sample_count = 0;

for i_alt = 1:N_alt
    h_uav = altitude_scenarios(i_alt);

    % Bu irtifa için UAV yörüngesini yeniden hesapla
    radius   = 500;
    center_x = 1000; center_y = 1000;
    theta    = linspace(0, 2*pi, N_pos+1);
    theta    = theta(1:end-1);
    UAV_pos  = zeros(N_pos, 3);
    UAV_pos(:,1) = center_x + radius*cos(theta');
    UAV_pos(:,2) = center_y + radius*sin(theta');
    UAV_pos(:,3) = h_uav;

    % TX-UAV mesafeleri
    d_uav = zeros(N_pos, 4);
    for pos = 1:N_pos
        for tx = 1:4
            d_uav(pos,tx) = norm(UAV_pos(pos,:) - TX(tx,:));
        end
    end

for i_spd = 1:N_speed
    UAV_spd = speed_scenarios(i_spd);

    % Hız vektörü
    UAV_vel = zeros(N_pos, 3);
    if UAV_spd > 0
        UAV_vel(:,1) = -UAV_spd * sin(theta');
        UAV_vel(:,2) =  UAV_spd * cos(theta');
    end

    % Doppler hesapla
    dop_hz = zeros(N_pos, 4);
    for pos = 1:N_pos
        for tx = 1:4
            vec = UAV_pos(pos,:) - TX(tx,:);
            d   = norm(vec);
            if d > 0 && UAV_spd > 0
                dop_hz(pos,tx) = dot(UAV_vel(pos,:), vec/d) / c * f_carrier;
            end
        end
    end

for i_env = 1:N_env
    env = env_scenarios(i_env);

for i_snr = 1:N_snr
    snr = SNR_values(i_snr);

    for pos = 1:N_pos
    for tx = 1:4

        % Gerçek gecikme [örnek]
        true_delay = d_uav(pos,tx) / c * Fs;
        delay_int  = round(true_delay);

        % Temel sinyal (yayılım gecikmesi)
        if delay_int < length(dab_frame)
            rx = [zeros(1,delay_int), dab_frame(1:end-delay_int)];
        else
            rx = zeros(size(dab_frame));
        end

        % Bozucuları uygula
        rx = add_multipath(rx, env.path_delays, env.path_powers, Fs);
        rx = add_doppler(rx, dop_hz(pos,tx), Fs);
        [rx, ~] = add_nlos(rx, env.nlos_prob, env.nlos_delay, env.nlos_loss, Fs);
        rx = add_shadowing(rx, env.shadow_std);
        rx = add_motor_noise(rx, motor_noise_power_dB, motor_pulse_prob);
        rx = add_clock(rx, 50, 10, Fs);
        rx = add_awgn(rx, snr);
        rx = add_quant(rx, 12);

        % Kirli korelasyon profili
        [toa_est, dirty_prof] = gcc_phat(rx, prs_symbol, N_null, N_prs);

        % Profili sabit uzunluğa getir
        if length(dirty_prof) >= corr_len
            dirty_prof = dirty_prof(1:corr_len);
        else
            dirty_prof(end+1:corr_len) = 0;
        end

        % Temiz (hedef) profil
        clean_prof = make_clean_profile(true_delay, N_prs, corr_len);

        % TOA hatası [m]
        toa_err_m = abs(toa_est - true_delay) * c / Fs;

        % Kaydet
        sample_count = sample_count + 1;
        all_dirty{end+1}  = dirty_prof;
        all_clean{end+1}  = clean_prof;
        all_toa_err(end+1)= toa_err_m;

        % Metadata
        all_labels{end+1} = struct(...
            'altitude',    h_uav, ...
            'speed',       UAV_spd, ...
            'environment', env.name, ...
            'snr_dB',      snr, ...
            'pos',         pos, ...
            'tx',          tx, ...
            'true_delay',  true_delay, ...
            'toa_est',     toa_est, ...
            'toa_err_m',   toa_err_m);

    end % tx
    end % pos

end % snr
end % env
end % speed

    fprintf('  İrtifa %dm tamamlandı → %d örnek\n', h_uav, sample_count);
end % altitude

fprintf('\n✓ Toplam %d örnek üretildi.\n\n', sample_count);

% =========================================================================
% BÖLÜM 4: MATRİS FORMATINA ÇEVİR
% =========================================================================
% Hücre dizisini matrise çevir (MATLAB Deep Learning için gerekli)

fprintf('Matris formatına dönüştürülüyor...\n');

X_dirty = zeros(sample_count, corr_len, 'single');   % Giriş (kirli)
Y_clean = zeros(sample_count, corr_len, 'single');   % Hedef (temiz)

for i = 1:sample_count
    X_dirty(i,:) = single(all_dirty{i});
    Y_clean(i,:) = single(all_clean{i});
end

fprintf('  X_dirty boyutu: %d × %d\n', size(X_dirty));
fprintf('  Y_clean boyutu: %d × %d\n\n', size(Y_clean));

% =========================================================================
% BÖLÜM 5: TRAIN / VAL / TEST BÖLME
% =========================================================================
% %70 eğitim, %15 doğrulama, %15 test

fprintf('Veri bölünüyor...\n');

rng(42);   % Tekrarlanabilirlik için
idx       = randperm(sample_count);

n_train   = floor(0.70 * sample_count);
n_val     = floor(0.15 * sample_count);
n_test    = sample_count - n_train - n_val;

idx_train = idx(1:n_train);
idx_val   = idx(n_train+1 : n_train+n_val);
idx_test  = idx(n_train+n_val+1 : end);

X_train = X_dirty(idx_train, :);
Y_train = Y_clean(idx_train, :);
X_val   = X_dirty(idx_val,   :);
Y_val   = Y_clean(idx_val,   :);
X_test  = X_dirty(idx_test,  :);
Y_test  = Y_clean(idx_test,  :);

toa_err_train = all_toa_err(idx_train);
toa_err_val   = all_toa_err(idx_val);
toa_err_test  = all_toa_err(idx_test);

fprintf('  Eğitim  : %d örnek (%%70)\n', n_train);
fprintf('  Doğrulama: %d örnek (%%15)\n', n_val);
fprintf('  Test    : %d örnek (%%15)\n\n', n_test);

% =========================================================================
% BÖLÜM 6: KAYDET
% =========================================================================

fprintf('Veriler kaydediliyor...\n');

save('dae_training_data.mat', ...
    'X_train', 'Y_train', ...
    'X_val',   'Y_val', ...
    'X_test',  'Y_test', ...
    'toa_err_train', 'toa_err_val', 'toa_err_test', ...
    'corr_len', 'sample_count', ...
    'altitude_scenarios', 'speed_scenarios', ...
    'SNR_values', ...
    '-v7.3');   % Büyük dosya için v7.3 formatı

fprintf('✓ "dae_training_data.mat" kaydedildi.\n\n');

% =========================================================================
% BÖLÜM 7: GRAFİKLER
% =========================================================================

figure('Name', 'Adım 9: DAE Eğitim Verisi', 'NumberTitle', 'off', ...
       'Position', [30, 30, 1300, 900]);

% --- Panel 1: Örnek Kirli vs Temiz Profiller ---
subplot(2, 3, 1);
hold on;
% 4 farklı SNR için kirli profil
snr_colors = {'r', 'm', 'b', 'c', 'g'};
for s = 1:min(3, N_snr)
    % O SNR'ye ait ilk örneği bul
    idx_snr = find(arrayfun(@(x) x.snr_dB == SNR_values(s) && ...
                             x.altitude == 200 && x.speed == 0, ...
                             [all_labels{:}]), 1);
    if ~isempty(idx_snr)
        plot(1:150, X_dirty(idx_snr, 1:150), ...
             'Color', snr_colors{s}, 'LineWidth', 1, ...
             'DisplayName', sprintf('Kirli SNR=%ddB', SNR_values(s)));
    end
end
plot(1:150, Y_clean(1, 1:150), 'k-', 'LineWidth', 2.5, ...
     'DisplayName', 'Temiz (Hedef)');
xlabel('Örnek'); ylabel('Norm. Korelasyon');
title('Kirli vs Temiz Profiller', 'FontWeight', 'bold');
legend('FontSize', 7, 'Location', 'northeast');
grid on;

% --- Panel 2: İrtifa Bazlı TOA Hatası ---
subplot(2, 3, 2);
toa_by_alt = zeros(N_alt, 1);
for ia = 1:N_alt
    mask = arrayfun(@(x) x.altitude == altitude_scenarios(ia), [all_labels{:}]);
    toa_by_alt(ia) = mean(all_toa_err(mask));
end
bar(toa_by_alt, 'FaceColor', [0.2 0.5 0.8]);
set(gca, 'XTickLabel', arrayfun(@(x) sprintf('%dm',x), altitude_scenarios, ...
         'UniformOutput', false));
xlabel('UAV İrtifası'); ylabel('Ort. TOA Hatası [m]');
title('İrtifaya Göre TOA Hatası', 'FontWeight', 'bold');
grid on;

% --- Panel 3: Hız Bazlı TOA Hatası ---
subplot(2, 3, 3);
toa_by_spd = zeros(N_speed, 1);
for is = 1:N_speed
    mask = arrayfun(@(x) x.speed == speed_scenarios(is), [all_labels{:}]);
    toa_by_spd(is) = mean(all_toa_err(mask));
end
bar(toa_by_spd, 'FaceColor', [0.8 0.4 0.2]);
set(gca, 'XTickLabel', speed_labels);
xlabel('UAV Hızı'); ylabel('Ort. TOA Hatası [m]');
title('Hıza Göre TOA Hatası', 'FontWeight', 'bold');
grid on;

% --- Panel 4: Ortam Bazlı TOA Hatası ---
subplot(2, 3, 4);
toa_by_env = zeros(N_env, 1);
env_names_list = {env_scenarios.name};
for ie = 1:N_env
    mask = arrayfun(@(x) strcmp(x.environment, env_names_list{ie}), [all_labels{:}]);
    toa_by_env(ie) = mean(all_toa_err(mask));
end
bar(toa_by_env, 'FaceColor', [0.6 0.2 0.6]);
set(gca, 'XTickLabel', env_names_list);
xlabel('Ortam'); ylabel('Ort. TOA Hatası [m]');
title('Ortama Göre TOA Hatası', 'FontWeight', 'bold');
grid on;

% --- Panel 5: SNR Bazlı TOA Hatası ---
subplot(2, 3, 5);
toa_by_snr = zeros(N_snr, 1);
for is = 1:N_snr
    mask = arrayfun(@(x) x.snr_dB == SNR_values(is), [all_labels{:}]);
    toa_by_snr(is) = mean(all_toa_err(mask));
end
plot(SNR_values, toa_by_snr, 'go-', 'LineWidth', 2, 'MarkerSize', 10);
xlabel('SNR [dB]'); ylabel('Ort. TOA Hatası [m]');
title('SNR''ye Göre TOA Hatası', 'FontWeight', 'bold');
grid on;

% --- Panel 6: Dataset Özeti ---
subplot(2, 3, 6);
axis off;
text(0.5, 0.98, 'DAE Dataset Özeti', 'HorizontalAlignment', 'center', ...
     'FontSize', 13, 'FontWeight', 'bold', 'Units', 'normalized');

ozet = {
    sprintf('Toplam örnek    : %d', sample_count);
    sprintf('Profil uzunluğu : %d', corr_len);
    '';
    sprintf('Eğitim seti     : %d (%%70)', n_train);
    sprintf('Doğrulama seti  : %d (%%15)', n_val);
    sprintf('Test seti       : %d (%%15)', n_test);
    '';
    'Kapsanan senaryolar:';
    sprintf('  İrtifa: %s m', num2str(altitude_scenarios));
    sprintf('  Hız: %s m/s', num2str(speed_scenarios));
    sprintf('  SNR: %s dB', num2str(SNR_values));
    '  Ortam: Kentsel, Karma';
    '  + Motor gürültüsü';
    '  + Doppler etkisi';
    '';
    sprintf('Ort. TOA hatası : %.1f m', mean(all_toa_err));
};

for i = 1:length(ozet)
    text(0.05, 0.91-(i-1)*0.058, ozet{i}, ...
         'FontSize', 9, 'Units', 'normalized');
end

sgtitle('ADIM 9: DAE Eğitim Verisi - Zengin Dataset', ...
        'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, 'step9_dataset.png');
fprintf('✓ Grafik kaydedildi.\n');

fprintf('\n=============================================\n');
fprintf('  ADIM 9 TAMAMLANDI ✓\n');
fprintf('=============================================\n');
fprintf('  Toplam örnek     : %d\n', sample_count);
fprintf('  Eğitim           : %d\n', n_train);
fprintf('  Ortalama TOA err : %.1f m\n', mean(all_toa_err));
fprintf('\nSonraki adım: Adım 10 - DAE Modeli ve Eğitimi\n');
fprintf('=============================================\n');