% =========================================================================
% ADIM 4: VARIŞ ZAMANI (TOA) KESTİRİMİ
% DAB SoOP TDOA Localization 
% =========================================================================
% Bu adımda iki aşamalı TOA kestirimi yapıyoruz:
%   Aşama 1 (Kaba): Null sembolünü enerji tabanlı dedektörle bul
%   Aşama 2 (Hassas): PRS çapraz korelasyonu ile hassas zamanlama
%
% Adım 3'teki TOA kestirimi basitti. Bu adımda daha gerçekçi bir
% dedektör zinciri kuruyoruz ve performansı analiz ediyoruz.
% =========================================================================

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('  ADIM 4: TOA Kestirimi Başlıyor...\n');
fprintf('========================================\n\n');

% =========================================================================
% BÖLÜM 1: VERİLERİ YÜKLE
% =========================================================================

load('geometry_data.mat');
load('dab_signal_data.mat');
load('channel_data.mat');

fprintf('✓ Tüm veriler yüklendi.\n\n');

% =========================================================================
% BÖLÜM 2: AŞAMA 1 - KABA ZAMANLA (NULL SEMBOLÜ DEDEKTÖRܾ)
% =========================================================================
% Null sembolü = enerji sıfır (sessizlik).
% Kayan pencere ile sinyalin enerjisini hesaplıyoruz.
% Enerji aniden düşerse → Null sembolü başlamış demektir.
%
% Yöntem: Sliding Window Energy Detector (Kayan Pencere Enerji Dedektörü)
%   - Pencere boyutu = Null sembol uzunluğu
%   - Her penceredeki ortalama enerjiyi hesapla
%   - Minimum enerji noktası = Null sembolünün yeri

    function null_idx = detect_null(rx_signal, n_null)
        % Kayan pencere enerjisi hesapla
        sig_len    = length(rx_signal);
        energy     = zeros(1, sig_len - n_null + 1);

        for k = 1:(sig_len - n_null + 1)
            window        = rx_signal(k : k + n_null - 1);
            energy(k)     = mean(abs(window).^2);
        end

        % Minimum enerji noktası = Null sembolünün başlangıcı
        [~, null_idx] = min(energy);
    end

% =========================================================================
% BÖLÜM 3: AŞAMA 2 - HASSAS ZAMANLA (PRS KORELASYONU)
% =========================================================================
% Null dedektörü yaklaşık çerçeve başını bulur (~birkaç sembol hata payı).
% PRS korelasyonu ile tam örnek hassasiyetinde TOA buluruz.
%
% GCC-PHAT (Generalized Cross Correlation - Phase Transform):
%   Standart korelasyona göre multipath ortamında daha iyi çalışır.
%   Frekans domeninde faz bilgisini kullanır, genliği normalize eder.

    function [toa_phat, corr_profile] = gcc_phat(rx_signal, prs_ref, n_null, n_prs)
        % PRS bölgesini çıkar
        start_idx  = n_null + 1;
        end_idx    = min(start_idx + n_prs - 1, length(rx_signal));
        prs_region = rx_signal(start_idx:end_idx);

        % Uzunlukları eşitle
        min_len    = min(length(prs_region), length(prs_ref));
        prs_region = prs_region(1:min_len);
        prs_ref_tr = prs_ref(1:min_len);

        % FFT al
        NFFT   = 2^nextpow2(2*min_len - 1);   % En yakın 2'nin kuvveti
        R1     = fft(prs_region, NFFT);         % Alınan sinyalin FFT'si
        R2     = fft(prs_ref_tr, NFFT);         % Referans PRS'in FFT'si

        % Çapraz güç spektrumu
        G      = R1 .* conj(R2);

        % PHAT ağırlıklandırma: Sadece faz bilgisini kullan, genliği normalize et
        % Bu sayede multipath bileşenleri bastırılır
        G_phat = G ./ (abs(G) + 1e-10);

        % IFFT ile korelasyon profilini al
        corr_ifft    = real(ifft(G_phat, NFFT));
        corr_profile = abs(corr_ifft);

        % Normalize et
        corr_profile = corr_profile / (max(corr_profile) + 1e-10);

        % Tepe noktasını bul (sadece pozitif lag tarafına bak)
        half          = floor(NFFT/2);
        [~, peak_idx] = max(corr_profile(1:half));

        % TOA = null uzunluğu + peak index - 1
        toa_phat = n_null + peak_idx - 1;
    end

% =========================================================================
% BÖLÜM 4: STANDART KORELASYON FONKSİYONU (Karşılaştırma için)
% =========================================================================

    function [toa_std, corr_profile] = standard_corr(rx_signal, prs_ref, n_null, n_prs)
        start_idx  = n_null + 1;
        end_idx    = min(start_idx + n_prs - 1, length(rx_signal));
        prs_region = rx_signal(start_idx:end_idx);

        min_len    = min(length(prs_region), length(prs_ref));
        prs_region = prs_region(1:min_len);
        prs_ref_tr = prs_ref(1:min_len);

        corr_raw     = xcorr(prs_region, prs_ref_tr);
        corr_profile = abs(corr_raw) / (max(abs(corr_raw)) + 1e-10);

        n_c          = length(prs_region);
        lags         = -(n_c-1):(n_c-1);
        [~, peak_idx]= max(corr_profile);
        toa_std      = n_null + lags(peak_idx);
    end

% =========================================================================
% BÖLÜM 5: SNR TARAMASI - FARKLI GÜRÜLTÜ SEVİYELERİNDE TEST
% =========================================================================
% DAE için eğitim verisi üretmek amacıyla farklı SNR'lerde
% TOA hatalarını hesaplıyoruz.

fprintf('SNR taraması yapılıyor...\n');

SNR_values     = [-5, 0, 5, 10, 15, 20, 25, 30];  % Test edilecek SNR'ler [dB]
N_trials       = 50;    % Her SNR seviyesi için tekrar sayısı
n_snr          = length(SNR_values);

% Sonuç matrisleri
toa_err_std_m  = zeros(n_snr, N_trials);   % Standart korelasyon hatası [m]
toa_err_phat_m = zeros(n_snr, N_trials);   % GCC-PHAT hatası [m]

% Multipath parametreleri (Adım 3'ten)
path_delays_samples = round([0, 0.5, 1.2] * 1e-6 * Fs);
path_powers_lin     = 10.^([0, -8, -15] / 10);

% Gerçek gecikme: TX1'den UAV'ın 1. konumuna (test için sabit)
true_delay_samples  = d_TX_UAV(1, 1) / c * Fs;

for s = 1:n_snr
    for trial = 1:N_trials
        % Test sinyali oluştur
        delay_int = round(true_delay_samples);
        if delay_int < length(dab_frame)
            rx = [zeros(1, delay_int), dab_frame(1:end-delay_int)];
        else
            rx = zeros(size(dab_frame));
        end

        % Multipath + AWGN ekle
        rx = add_multipath_local(rx, path_delays_samples, path_powers_lin);
        rx = add_awgn_local(rx, SNR_values(s));

        % Standart korelasyon
        [toa_std, ~]  = standard_corr(rx, prs_symbol, N_null, N_prs);
        err_std       = abs(toa_std - true_delay_samples) * c / Fs;
        toa_err_std_m(s, trial) = err_std;

        % GCC-PHAT
        [toa_ph, ~]   = gcc_phat(rx, prs_symbol, N_null, N_prs);
        err_phat      = abs(toa_ph - true_delay_samples) * c / Fs;
        toa_err_phat_m(s, trial) = err_phat;
    end
    fprintf('  SNR=%+3ddB → Std: %.1fm | PHAT: %.1fm\n', ...
            SNR_values(s), mean(toa_err_std_m(s,:)), mean(toa_err_phat_m(s,:)));
end

fprintf('\n');

% =========================================================================
% BÖLÜM 6: TÜM KONUMLAR İÇİN GCC-PHAT İLE TOA KESİTİRİMİ
% =========================================================================
% Adım 3'teki TOA_UAV'ı daha iyi GCC-PHAT yöntemiyle güncelle

fprintf('GCC-PHAT ile tüm konumlar için TOA kestirimi...\n');

TOA_UAV_phat   = zeros(N_pos, 4);
TOA_RefRX_phat = zeros(1, 4);

% Adım 3'teki aynı kanal parametrelerini kullan
clock_offset_ns = 50;
clock_drift_ppb = 10;
ADC_bits        = 12;
SNR_dB          = 15;

% Ref RX
for tx = 1:4
    delay_int = round(d_TX_RefRX(tx) / c * Fs);
    if delay_int < length(dab_frame)
        rx = [zeros(1, delay_int), dab_frame(1:end-delay_int)];
    else
        rx = zeros(size(dab_frame));
    end
    rx = add_multipath_local(rx, path_delays_samples, path_powers_lin);
    rx = add_clock_error_local(rx, clock_offset_ns, clock_drift_ppb, Fs);
    rx = add_awgn_local(rx, SNR_dB);
    rx = add_quantization_local(rx, ADC_bits);
    TOA_RefRX_phat(tx) = gcc_phat(rx, prs_symbol, N_null, N_prs);
end

% UAV
for pos = 1:N_pos
    for tx = 1:4
        delay_int = round(d_TX_UAV(pos,tx) / c * Fs);
        if delay_int < length(dab_frame)
            rx = [zeros(1, delay_int), dab_frame(1:end-delay_int)];
        else
            rx = zeros(size(dab_frame));
        end
        rx = add_multipath_local(rx, path_delays_samples, path_powers_lin);
        rx = add_clock_error_local(rx, clock_offset_ns, clock_drift_ppb, Fs);
        rx = add_awgn_local(rx, SNR_dB);
        rx = add_quantization_local(rx, ADC_bits);
        TOA_UAV_phat(pos, tx) = gcc_phat(rx, prs_symbol, N_null, N_prs);
    end
end

fprintf('✓ GCC-PHAT TOA kestirimi tamamlandı.\n\n');

% Hata analizi
TOA_err_phat_UAV    = TOA_UAV_phat - TOA_true_UAV;
TOA_err_phat_UAV_m  = abs(TOA_err_phat_UAV) * c / Fs;

fprintf('GCC-PHAT TOA Hata Analizi:\n');
fprintf('  Ortalama hata: %.2f m\n', mean(TOA_err_phat_UAV_m(:)));
fprintf('  Std sapma    : %.2f m\n', std(TOA_err_phat_UAV_m(:)));
fprintf('  Maks hata    : %.2f m\n\n', max(TOA_err_phat_UAV_m(:)));

% =========================================================================
% BÖLÜM 7: KAYDET
% =========================================================================

save('toa_data.mat', ...
    'TOA_UAV_phat', ...
    'TOA_RefRX_phat', ...
    'TOA_err_phat_UAV', ...
    'TOA_err_phat_UAV_m', ...
    'TOA_true_UAV', ...
    'TOA_true_RefRX', ...
    'SNR_values', ...
    'toa_err_std_m', ...
    'toa_err_phat_m');

fprintf('✓ Veriler "toa_data.mat" dosyasına kaydedildi.\n\n');

% =========================================================================
% BÖLÜM 8: GRAFİKLER
% =========================================================================

figure('Name', 'Adım 4: TOA Kestirimi', 'NumberTitle', 'off', ...
       'Position', [50, 50, 1200, 800]);

% Panel 1: SNR vs TOA Hatası (Std vs PHAT)
subplot(2, 3, 1);
mean_std  = mean(toa_err_std_m, 2);
mean_phat = mean(toa_err_phat_m, 2);
plot(SNR_values, mean_std,  'r-o', 'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', 'Standart Korelasyon');
hold on;
plot(SNR_values, mean_phat, 'b-s', 'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', 'GCC-PHAT');
xlabel('SNR [dB]', 'FontSize', 10);
ylabel('TOA Hatası [m]', 'FontSize', 10);
title('SNR vs TOA Hatası', 'FontWeight', 'bold');
legend('FontSize', 9, 'Location', 'northeast');
grid on;

% Panel 2: Korelasyon profili karşılaştırması (SNR=5dB örneği)
subplot(2, 3, 2);
delay_int_test = round(true_delay_samples);
if delay_int_test < length(dab_frame)
    rx_test = [zeros(1, delay_int_test), dab_frame(1:end-delay_int_test)];
else
    rx_test = zeros(size(dab_frame));
end
rx_test = add_multipath_local(rx_test, path_delays_samples, path_powers_lin);
rx_test = add_awgn_local(rx_test, 5);   % Düşük SNR test

[~, corr_std_profile]  = standard_corr(rx_test, prs_symbol, N_null, N_prs);
[~, corr_phat_profile] = gcc_phat(rx_test, prs_symbol, N_null, N_prs);

n_show   = 200;
x_std    = (1:min(n_show, length(corr_std_profile)));
x_phat   = (1:min(n_show, length(corr_phat_profile)));
plot(x_std,  corr_std_profile(x_std),  'r-', 'LineWidth', 1.5, ...
     'DisplayName', 'Standart');
hold on;
plot(x_phat, corr_phat_profile(x_phat), 'b-', 'LineWidth', 1.5, ...
     'DisplayName', 'GCC-PHAT');
xlabel('Örnek', 'FontSize', 10);
ylabel('Norm. Korelasyon', 'FontSize', 10);
title('Korelasyon Profili (SNR=5dB)', 'FontWeight', 'bold');
legend('FontSize', 9);
grid on;

% Panel 3: GCC-PHAT TOA hata histogramı
subplot(2, 3, 3);
histogram(TOA_err_phat_UAV_m(:), 15, 'FaceColor', [0.2 0.6 1], ...
          'EdgeColor', 'white');
xline(mean(TOA_err_phat_UAV_m(:)), 'r--', 'LineWidth', 2, ...
      'Label', sprintf('Ort=%.1fm', mean(TOA_err_phat_UAV_m(:))));
xlabel('TOA Hatası [m]', 'FontSize', 10);
ylabel('Frekans', 'FontSize', 10);
title('GCC-PHAT TOA Hata Dağılımı', 'FontWeight', 'bold');
grid on;

% Panel 4: TX bazlı TOA hatası
subplot(2, 3, 4);
tx_mean_err = mean(TOA_err_phat_UAV_m, 1);
tx_std_err  = std(TOA_err_phat_UAV_m, 0, 1);
bar(tx_mean_err, 'FaceColor', [0.4 0.7 0.3]);
hold on;
errorbar(1:4, tx_mean_err, tx_std_err, 'k.', 'LineWidth', 2);
set(gca, 'XTickLabel', {'TX1','TX2','TX3','TX4'});
xlabel('Verici', 'FontSize', 10);
ylabel('Ort. TOA Hatası [m]', 'FontSize', 10);
title('TX Bazlı TOA Hatası', 'FontWeight', 'bold');
grid on;

% Panel 5: UAV konumu bazlı TOA hatası
subplot(2, 3, 5);
pos_mean_err = mean(TOA_err_phat_UAV_m, 2);
plot(1:N_pos, pos_mean_err, 'mo-', 'LineWidth', 2, 'MarkerSize', 7);
xlabel('UAV Konumu', 'FontSize', 10);
ylabel('Ort. TOA Hatası [m]', 'FontSize', 10);
title('UAV Konumuna Göre TOA Hatası', 'FontWeight', 'bold');
grid on;
xlim([1, N_pos]);

% Panel 6: Kümülatif hata dağılımı (CDF)
subplot(2, 3, 6);
err_sorted = sort(TOA_err_phat_UAV_m(:));
cdf_vals   = (1:length(err_sorted)) / length(err_sorted);
plot(err_sorted, cdf_vals * 100, 'b-', 'LineWidth', 2);
xline(mean(err_sorted), 'r--', 'LineWidth', 1.5, ...
      'Label', sprintf('Ort=%.1fm', mean(err_sorted)));
xlabel('TOA Hatası [m]', 'FontSize', 10);
ylabel('Kümülatif Olasılık [%]', 'FontSize', 10);
title('TOA Hatası CDF', 'FontWeight', 'bold');
grid on;
ylim([0, 100]);

sgtitle('ADIM 4: TOA Kestirimi - Standart vs GCC-PHAT', ...
        'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, 'step4_toa.png');
fprintf('✓ Grafik "step4_toa.png" kaydedildi.\n');

fprintf('\n========================================\n');
fprintf('  ADIM 4 TAMAMLANDI ✓\n');
fprintf('========================================\n');
fprintf('Sonraki adım: step5_tdoa.m\n');
fprintf('========================================\n');

% =========================================================================
% YEREL YARDIMCI FONKSİYONLAR (Adım 3'ten kopyalandı)
% =========================================================================

function noisy = add_awgn_local(signal, snr_dB)
    signal_power = mean(abs(signal).^2);
    snr_lin      = 10^(snr_dB / 10);
    noise_power  = signal_power / snr_lin;
    noisy        = signal + sqrt(noise_power) * randn(size(signal));
end

function mp = add_multipath_local(signal, delays, powers_lin)
    mp = zeros(size(signal));
    for p = 1:length(delays)
        d = delays(p);
        if d == 0
            mp = mp + sqrt(powers_lin(p)) * signal;
        else
            mp = mp + sqrt(powers_lin(p)) * [zeros(1,d), signal(1:end-d)];
        end
    end
end

function shifted = add_clock_error_local(signal, offset_ns, drift_ppb, fs)
    offset_int = round(offset_ns * 1e-9 * fs);
    if offset_int > 0
        shifted = [zeros(1,offset_int), signal(1:end-offset_int)];
    elseif offset_int < 0
        shifted = [signal(-offset_int+1:end), zeros(1,-offset_int)];
    else
        shifted = signal;
    end
    t       = (0:length(shifted)-1) / fs;
    shifted = real(shifted .* real(exp(1j*2*pi*drift_ppb*1e-9*fs*t)));
end

function quantized = add_quantization_local(signal, n_bits)
    sig_max = max(abs(signal));
    if sig_max == 0; quantized = signal; return; end
    step      = 2 / 2^n_bits;
    quantized = round((signal/sig_max) / step) * step * sig_max;
end