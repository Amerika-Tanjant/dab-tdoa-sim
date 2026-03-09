% =========================================================================
% ADIM 3 V2: GELİŞMİŞ KANAL MODELİ - DOPPLER + NLOS/GÖLGELEME
% DAB SoOP TDOA Localization
% =========================================================================
% Adım 3'e eklenen yeni etkiler:
%   5. Doppler kayması   → UAV hareketi frekans kaymasına yol açar
%   6. NLOS/Gölgeleme   → Bazı TX'ler binalar tarafından engellenir
%
% Bu dosya daha zengin ve zorlu bir eğitim dataseti üretir.
% Çıktı: 'channel_data_v2.mat'
% =========================================================================

clear all;
close all;
clc;

fprintf('=============================================\n');
fprintf('  ADIM 3 V2: Gelişmiş Kanal Modeli\n');
fprintf('  Doppler + NLOS/Gölgeleme Eklendi\n');
fprintf('=============================================\n\n');

% =========================================================================
% BÖLÜM 1: VERİLERİ YÜKLE
% =========================================================================

load('geometry_data.mat');
load('dab_signal_data.mat');

fprintf('✓ Geometri ve sinyal verileri yüklendi.\n\n');

% =========================================================================
% BÖLÜM 2: TEMEL KANAL PARAMETRELERİ (Adım 3'ten aynı)
% =========================================================================

SNR_dB_base     = 15;
path_delays_us  = [0, 0.5, 1.2];
path_powers_dB  = [0, -8, -15];
clock_offset_ns = 50;
clock_drift_ppb = 10;
ADC_bits        = 12;

path_delays_samples = round(path_delays_us * 1e-6 * Fs);
path_powers_lin     = 10.^(path_powers_dB / 10);

% =========================================================================
% BÖLÜM 3: DOPPLER PARAMETRELERİ
% =========================================================================
% UAV dairesel yörüngede hareket ediyor.
% Her konumda TX'e göre yaklaşma/uzaklaşma hızı farklı.
%
% Doppler kayması formülü:
%   Δf = (v_radyal / c) × f_carrier
%
%   v_radyal: UAV'ın TX'e göre radyal hızı (yaklaşma → pozitif)
%   f_carrier: DAB taşıyıcı frekansı
%   c: ışık hızı

UAV_speed     = 30;          % UAV hızı [m/s] = ~108 km/h (gerçekçi)
f_carrier     = 194e6;       % DAB Band-III merkez frekansı [Hz] (~194 MHz)

% UAV'ın hız vektörünü hesapla (dairesel yörüngede tanjant yön)
% Dairesel hareket: hız vektörü merkeze dik (tanjant)
% theta: UAV'ın açısal konumu
theta_vals = linspace(0, 2*pi, N_pos+1);
theta_vals = theta_vals(1:end-1);

% Tanjant hız vektörü (dairesel yörünge için):
%   vx = -v * sin(theta)
%   vy =  v * cos(theta)
UAV_velocity = zeros(N_pos, 3);
UAV_velocity(:,1) = -UAV_speed * sin(theta_vals');  % vx
UAV_velocity(:,2) =  UAV_speed * cos(theta_vals');  % vy
UAV_velocity(:,3) = 0;                               % vz = 0 (sabit irtifa)

fprintf('Doppler Parametreleri:\n');
fprintf('  UAV hızı         : %.0f m/s (%.0f km/h)\n', UAV_speed, UAV_speed*3.6);
fprintf('  Taşıyıcı frekans : %.0f MHz\n', f_carrier/1e6);
fprintf('  Maks Doppler kayması: %.2f Hz\n\n', UAV_speed/c * f_carrier);

% =========================================================================
% BÖLÜM 4: NLOS / GÖLGELEME PARAMETRELERİ
% =========================================================================
% Gerçek şehir ortamında bazı TX'ler binalar tarafından engellenir.
%
% İki senaryo:
%   LOS (Line of Sight): Direkt görüş hattı var → normal sinyal
%   NLOS (Non-Line of Sight): Bina engeli var → ekstra gecikme + zayıflama
%
% NLOS modeli:
%   Ekstra gecikme: 50-500 ns (sinyal binadan geçiyor/yansıyor)
%   Ekstra zayıflama: 10-30 dB (bina duvarı geçişi)
%   Olasılık: Her TX için bağımsız, %30 NLOS olasılığı

NLOS_probability  = 0.3;    % Her TX'in NLOS olma olasılığı
NLOS_delay_min_ns = 50;     % Min ekstra gecikme [ns]
NLOS_delay_max_ns = 500;    % Max ekstra gecikme [ns]
NLOS_loss_min_dB  = 10;     % Min ekstra kayıp [dB]
NLOS_loss_max_dB  = 30;     % Max ekstra kayıp [dB]

% Log-normal gölgeleme (shadowing)
% Bina yoğunluğuna göre rastgele sinyal dalgalanması
sigma_shadow_dB = 8;        % Gölgeleme standart sapması [dB]
                            % Şehir içi için tipik değer: 6-10 dB

fprintf('NLOS/Gölgeleme Parametreleri:\n');
fprintf('  NLOS olasılığı   : %.0f%%\n', NLOS_probability*100);
fprintf('  NLOS ekstra gecikme: %d-%d ns\n', NLOS_delay_min_ns, NLOS_delay_max_ns);
fprintf('  NLOS ekstra kayıp  : %d-%d dB\n', NLOS_loss_min_dB, NLOS_loss_max_dB);
fprintf('  Gölgeleme std    : %.0f dB\n\n', sigma_shadow_dB);

% =========================================================================
% BÖLÜM 5: DOPPLER UYGULAMA FONKSİYONU
% =========================================================================
% Doppler etkisi: frekans kayması → zaman domeninde faz dönüşü
%
% Matematiksel ifade:
%   s_doppler(t) = s(t) × exp(j × 2π × Δf × t)
%
% Bu çarpım, sinyalin her örneğine artan faz ekler.
% Korelasyon tepesini kaydırır ve genişletir → TOA hatası artar.

    function rx_doppler = apply_doppler(signal, doppler_hz, fs)
        N  = length(signal);
        t  = (0:N-1) / fs;                          % Zaman ekseni
        % Karmaşık faz dönüşü (baseband Doppler modeli)
        phase_shift = exp(1j * 2 * pi * doppler_hz * t);
        % Gerçek kısım al (bandpass sinyal modeli)
        rx_doppler  = real(signal .* real(phase_shift));
    end

% =========================================================================
% BÖLÜM 6: NLOS UYGULAMA FONKSİYONU
% =========================================================================
% NLOS durumunda:
%   1. Ekstra gecikme ekle (bina geçiş süresi)
%   2. Sinyali zayıflat (duvar kaybı)
%   3. Ek multipath bileşenleri ekle (yansıma yolları)

    function [rx_nlos, is_nlos, nlos_delay_ns] = apply_nlos(signal, ...
             nlos_prob, delay_min, delay_max, loss_min, loss_max, fs, c_light)

        is_nlos      = rand() < nlos_prob;   % NLOS mu?
        nlos_delay_ns = 0;

        if is_nlos
            % Ekstra gecikme [örnek]
            extra_delay_ns      = delay_min + rand() * (delay_max - delay_min);
            extra_delay_samples = round(extra_delay_ns * 1e-9 * fs);
            nlos_delay_ns       = extra_delay_ns;

            % Ekstra kayıp [lineer]
            extra_loss_dB  = loss_min + rand() * (loss_max - loss_min);
            extra_loss_lin = 10^(-extra_loss_dB / 10);

            % Sinyali geciktir ve zayıflat
            if extra_delay_samples > 0 && extra_delay_samples < length(signal)
                rx_nlos = sqrt(extra_loss_lin) * ...
                          [zeros(1, extra_delay_samples), ...
                           signal(1:end-extra_delay_samples)];
            else
                rx_nlos = sqrt(extra_loss_lin) * signal;
            end
        else
            rx_nlos = signal;   % LOS: değişiklik yok
        end
    end

% =========================================================================
% BÖLÜM 7: TEMEL YARDIMCI FONKSİYONLAR
% =========================================================================

    function noisy = add_awgn(signal, snr_dB)
        sp    = mean(abs(signal).^2);
        nl    = sp / 10^(snr_dB/10);
        noisy = signal + sqrt(nl) * randn(size(signal));
    end

    function mp = add_multipath(signal, delays, powers_lin)
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

    function shifted = add_clock_error(signal, offset_ns, drift_ppb, fs)
        oi = round(offset_ns * 1e-9 * fs);
        if oi > 0
            shifted = [zeros(1,oi), signal(1:end-oi)];
        elseif oi < 0
            shifted = [signal(-oi+1:end), zeros(1,-oi)];
        else
            shifted = signal;
        end
        t       = (0:length(shifted)-1) / fs;
        shifted = real(shifted .* real(exp(1j*2*pi*drift_ppb*1e-9*fs*t)));
    end

    function q = add_quantization(signal, n_bits)
        sm = max(abs(signal));
        if sm == 0; q = signal; return; end
        step = 2 / 2^n_bits;
        q    = round((signal/sm) / step) * step * sm;
    end

    function [toa, corr_prof] = estimate_toa_phat(rx, prs_ref, n_null, n_prs)
        s   = n_null + 1;
        e   = min(s + n_prs - 1, length(rx));
        seg = rx(s:e);
        ml  = min(length(seg), length(prs_ref));
        seg = seg(1:ml);
        ref = prs_ref(1:ml);
        N2  = 2^nextpow2(2*ml-1);
        G   = fft(seg,N2) .* conj(fft(ref,N2));
        G   = G ./ (abs(G) + 1e-10);
        cr  = real(ifft(G,N2));
        corr_prof = abs(cr) / (max(abs(cr)) + 1e-10);
        [~,pk] = max(corr_prof(1:floor(N2/2)));
        toa    = n_null + pk - 1;
    end

% =========================================================================
% BÖLÜM 8: DOPPLER KAYMASINI HESAPLA
% =========================================================================
% Her UAV konumu ve TX çifti için radyal hızı hesapla

doppler_hz = zeros(N_pos, 4);   % Doppler frekans kayması [Hz]

for pos = 1:N_pos
    for tx = 1:4
        % TX'ten UAV'a birim yön vektörü
        vec_tx_to_uav = UAV_true(pos,:) - TX(tx,:);
        dist          = norm(vec_tx_to_uav);
        if dist < 1e-6; continue; end
        unit_vec      = vec_tx_to_uav / dist;

        % Radyal hız: hız vektörünün TX yönündeki bileşeni
        v_radyal      = dot(UAV_velocity(pos,:), unit_vec);

        % Doppler kayması: Δf = (v_radyal / c) × f_carrier
        doppler_hz(pos, tx) = (v_radyal / c) * f_carrier;
    end
end

fprintf('Doppler Kayması Özeti:\n');
fprintf('  Min: %.2f Hz\n', min(doppler_hz(:)));
fprintf('  Max: %.2f Hz\n', max(doppler_hz(:)));
fprintf('  Ort: %.2f Hz\n\n', mean(abs(doppler_hz(:))));

% =========================================================================
% BÖLÜM 9: TÜM SENARYOLAR İÇİN VERİ ÜRET
% =========================================================================
% 4 farklı senaryo → zengin DAE eğitim dataseti:
%   Senaryo 1: Temel (Adım 3 ile aynı)
%   Senaryo 2: Doppler ekli
%   Senaryo 3: NLOS/Gölgeleme ekli
%   Senaryo 4: Doppler + NLOS birlikte (en zorlu)

scenario_names = {'Temel', 'Doppler', 'NLOS', 'Doppler+NLOS'};
N_scenarios    = 4;
corr_len       = 2 * N_prs - 1;

% Tüm senaryolar için veri matrisleri
TOA_UAV_all    = zeros(N_scenarios, N_pos, 4);
TOA_RefRX_all  = zeros(N_scenarios, 4);
NLOS_flags_all = zeros(N_scenarios, N_pos, 4);   % Hangi TX NLOS?
NLOS_delay_all = zeros(N_scenarios, N_pos, 4);   % NLOS gecikmeleri
corr_profiles  = zeros(N_scenarios, N_pos, 4, corr_len);  % DAE eğitim verisi

fprintf('Tüm senaryolar işleniyor...\n\n');

for sc = 1:N_scenarios
    fprintf('Senaryo %d/%d: %s\n', sc, N_scenarios, scenario_names{sc});

    use_doppler = (sc == 2 || sc == 4);
    use_nlos    = (sc == 3 || sc == 4);

    % --- Ref RX ---
    for tx = 1:4
        delay_int = round(d_TX_RefRX(tx) / c * Fs);
        if delay_int < length(dab_frame)
            rx = [zeros(1,delay_int), dab_frame(1:end-delay_int)];
        else
            rx = zeros(size(dab_frame));
        end

        rx = add_multipath(rx, path_delays_samples, path_powers_lin);

        % NLOS (Ref RX için - sabit konum, düşük NLOS olasılığı)
        if use_nlos
            [rx, ~, ~] = apply_nlos(rx, NLOS_probability*0.5, ...
                NLOS_delay_min_ns, NLOS_delay_max_ns, ...
                NLOS_loss_min_dB,  NLOS_loss_max_dB, Fs, c);
        end

        % Gölgeleme (log-normal)
        if use_nlos
            shadow_dB  = sigma_shadow_dB * randn();
            shadow_lin = 10^(shadow_dB/20);
            rx         = rx * shadow_lin;
        end

        rx = add_clock_error(rx, clock_offset_ns, clock_drift_ppb, Fs);
        rx = add_awgn(rx, SNR_dB_base);
        rx = add_quantization(rx, ADC_bits);

        TOA_RefRX_all(sc, tx) = estimate_toa_phat(rx, prs_symbol, N_null, N_prs);
    end

    % --- UAV ---
    for pos = 1:N_pos
        for tx = 1:4
            delay_int = round(d_TX_UAV(pos,tx) / c * Fs);
            if delay_int < length(dab_frame)
                rx = [zeros(1,delay_int), dab_frame(1:end-delay_int)];
            else
                rx = zeros(size(dab_frame));
            end

            rx = add_multipath(rx, path_delays_samples, path_powers_lin);

            % DOPPLER UYGULA
            if use_doppler
                rx = apply_doppler(rx, doppler_hz(pos,tx), Fs);
            end

            % NLOS UYGULA
            nlos_flag = false;
            nlos_d_ns = 0;
            if use_nlos
                [rx, nlos_flag, nlos_d_ns] = apply_nlos(rx, ...
                    NLOS_probability, NLOS_delay_min_ns, NLOS_delay_max_ns, ...
                    NLOS_loss_min_dB, NLOS_loss_max_dB, Fs, c);

                % Log-normal gölgeleme
                shadow_dB  = sigma_shadow_dB * randn();
                shadow_lin = 10^(shadow_dB/20);
                rx         = rx * shadow_lin;
            end

            NLOS_flags_all(sc, pos, tx) = nlos_flag;
            NLOS_delay_all(sc, pos, tx) = nlos_d_ns;

            rx = add_clock_error(rx, clock_offset_ns, clock_drift_ppb, Fs);
            rx = add_awgn(rx, SNR_dB_base);
            rx = add_quantization(rx, ADC_bits);

            % TOA + Korelasyon profili
            [toa_val, corr_p] = estimate_toa_phat(rx, prs_symbol, N_null, N_prs);
            TOA_UAV_all(sc, pos, tx) = toa_val;

            len_p = length(corr_p);
            if len_p >= corr_len
                corr_profiles(sc, pos, tx, :) = corr_p(1:corr_len);
            else
                corr_profiles(sc, pos, tx, 1:len_p) = corr_p;
            end
        end
    end

    % NLOS istatistikleri
    if use_nlos
        nlos_count = sum(NLOS_flags_all(sc,:,:), 'all');
        fprintf('  NLOS olan TX sayısı: %d/%d (%.0f%%)\n', ...
                nlos_count, N_pos*4, nlos_count/(N_pos*4)*100);
    end
    fprintf('  ✓ Tamamlandı.\n');
end

fprintf('\n');

% =========================================================================
% BÖLÜM 10: TDOA VE HATA ANALİZİ
% =========================================================================

TOA_true_UAV_all   = zeros(N_pos, 4);
TOA_true_RefRX_all = zeros(1, 4);

for tx = 1:4
    TOA_true_RefRX_all(tx) = d_TX_RefRX(tx) / c * Fs;
    for pos = 1:N_pos
        TOA_true_UAV_all(pos,tx) = d_TX_UAV(pos,tx) / c * Fs;
    end
end

% Her senaryo için TOA hatası
fprintf('TOA Hata Analizi (ortalama mutlak hata [m]):\n');
fprintf('  %-20s  %-10s\n', 'Senaryo', 'Ort. Hata');
fprintf('  %-20s  %-10s\n', '--------------------', '----------');

toa_errors_m = zeros(N_scenarios, 1);
for sc = 1:N_scenarios
    err = squeeze(TOA_UAV_all(sc,:,:)) - TOA_true_UAV_all;
    toa_errors_m(sc) = mean(abs(err(:))) * c / Fs;
    fprintf('  %-20s  %-10.2f m\n', scenario_names{sc}, toa_errors_m(sc));
end

% =========================================================================
% BÖLÜM 11: KAYDET
% =========================================================================

save('channel_data_v2.mat', ...
    'TOA_UAV_all', ...          % [4 senaryo x 20 konum x 4 TX]
    'TOA_RefRX_all', ...        % [4 senaryo x 4 TX]
    'TOA_true_UAV_all', ...
    'TOA_true_RefRX_all', ...
    'corr_profiles', ...        % [4 x 20 x 4 x corr_len] ← DAE verisi!
    'NLOS_flags_all', ...       % Hangi TX NLOS oldu?
    'NLOS_delay_all', ...       % NLOS gecikme değerleri
    'doppler_hz', ...           % Doppler kaymaları
    'UAV_velocity', ...         % UAV hız vektörleri
    'scenario_names', ...
    'toa_errors_m', ...
    'N_scenarios');

fprintf('\n✓ Veriler "channel_data_v2.mat" dosyasına kaydedildi.\n');
fprintf('  Toplam korelasyon profili: %d x %d x %d x %d\n\n', ...
        N_scenarios, N_pos, 4, corr_len);

% =========================================================================
% BÖLÜM 12: GRAFİKLER
% =========================================================================

figure('Name', 'Adım 3 V2: Gelişmiş Kanal Modeli', 'NumberTitle', 'off', ...
       'Position', [30, 30, 1300, 900]);

% --- Panel 1: Doppler Kayması Haritası ---
subplot(2, 3, 1);
doppler_map = doppler_hz;   % N_pos x 4
imagesc(1:4, 1:N_pos, doppler_map);
colorbar;
colormap(gca, 'jet');
xlabel('TX', 'FontSize', 10);
ylabel('UAV Konumu', 'FontSize', 10);
title('Doppler Kayması [Hz]', 'FontWeight', 'bold');
set(gca, 'XTick', 1:4, 'XTickLabel', {'TX1','TX2','TX3','TX4'});

% --- Panel 2: UAV Hız Vektörleri ---
subplot(2, 3, 2);
hold on; grid on;
plot(UAV_true(:,1), UAV_true(:,2), 'k--', 'LineWidth', 1.5);
quiver(UAV_true(:,1), UAV_true(:,2), ...
       UAV_velocity(:,1)*20, UAV_velocity(:,2)*20, 0, ...
       'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
scatter(TX(:,1), TX(:,2), 100, 'rs', 'filled');
for m=1:4
    text(TX(m,1)+60, TX(m,2)+60, sprintf('TX%d',m), 'FontSize',8, 'Color','red');
end
xlabel('X [m]'); ylabel('Y [m]');
title('UAV Hız Vektörleri', 'FontWeight', 'bold');
axis equal;
xlim([-200, 2200]); ylim([-200, 2200]);

% --- Panel 3: Korelasyon Profili - 4 Senaryo Karşılaştırması ---
subplot(2, 3, 3);
colors = {'g', 'b', 'r', 'm'};
hold on;
for sc = 1:N_scenarios
    prof = squeeze(corr_profiles(sc, 1, 1, :));
    plot(1:200, prof(1:200), 'Color', colors{sc}, 'LineWidth', 1.5, ...
         'DisplayName', scenario_names{sc});
end
xlabel('Örnek', 'FontSize', 10);
ylabel('Norm. Korelasyon', 'FontSize', 10);
title('Korelasyon: 4 Senaryo (Konum1, TX1)', 'FontWeight', 'bold');
legend('FontSize', 8, 'Location', 'northeast');
grid on;

% --- Panel 4: TOA Hatası Karşılaştırması ---
subplot(2, 3, 4);
b = bar(toa_errors_m, 'FaceColor', 'flat');
b.CData(1,:) = [0.2 0.7 0.2];
b.CData(2,:) = [0.2 0.4 0.8];
b.CData(3,:) = [0.8 0.2 0.2];
b.CData(4,:) = [0.6 0.1 0.6];
set(gca, 'XTickLabel', scenario_names);
ylabel('Ort. TOA Hatası [m]', 'FontSize', 10);
title('Senaryo Bazlı TOA Hatası', 'FontWeight', 'bold');
grid on;

% --- Panel 5: NLOS Gecikme Dağılımı ---
subplot(2, 3, 5);
% Senaryo 3 (NLOS) ve Senaryo 4 (Doppler+NLOS) için
nlos_delays_sc3 = squeeze(NLOS_delay_all(3,:,:));
nlos_delays_sc4 = squeeze(NLOS_delay_all(4,:,:));
nlos_vals = [nlos_delays_sc3(:); nlos_delays_sc4(:)];
nlos_vals = nlos_vals(nlos_vals > 0);   % Sadece NLOS olanlar
if ~isempty(nlos_vals)
    histogram(nlos_vals, 15, 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'white');
end
xlabel('NLOS Ekstra Gecikme [ns]', 'FontSize', 10);
ylabel('Frekans', 'FontSize', 10);
title('NLOS Gecikme Dağılımı', 'FontWeight', 'bold');
grid on;

% --- Panel 6: DAE Veri Özeti ---
subplot(2, 3, 6);
axis off;
text(0.5, 0.95, 'DAE Eğitim Verisi Özeti', 'HorizontalAlignment', 'center', ...
     'FontSize', 12, 'FontWeight', 'bold', 'Units', 'normalized');

info = {
    sprintf('Toplam profil sayısı: %d', N_scenarios * N_pos * 4);
    sprintf('  (4 senaryo × %d konum × 4 TX)', N_pos);
    '';
    sprintf('Profil uzunluğu: %d nokta', corr_len);
    '';
    'Senaryolar:';
    '  1. Temel (referans)';
    '  2. + Doppler etkisi';
    '  3. + NLOS/Gölgeleme';
    '  4. + Doppler ve NLOS';
    '';
    '→ DAE tüm bu durumları';
    '  öğrenecek!'
};

for i = 1:length(info)
    text(0.05, 0.88 - (i-1)*0.07, info{i}, ...
         'FontSize', 10, 'Units', 'normalized');
end

sgtitle('ADIM 3 V2: Gelişmiş Kanal - Doppler + NLOS', ...
        'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, 'step3_v2_advanced_channel.png');
fprintf('✓ Grafik kaydedildi.\n');

fprintf('\n=============================================\n');
fprintf('  ADIM 3 V2 TAMAMLANDI ✓\n');
fprintf('=============================================\n');
fprintf('Üretilen DAE eğitim verisi:\n');
fprintf('  corr_profiles: [%d x %d x %d x %d]\n', ...
        N_scenarios, N_pos, 4, corr_len);
fprintf('  = %d korelasyon profili\n', N_scenarios*N_pos*4);
fprintf('\nSonraki adım: Adım 9 - DAE Eğitimi\n');
fprintf('=============================================\n');