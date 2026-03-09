% =========================================================================
% ADIM 8: PERFORMANS ANALİZİ VE SONUÇ RAPORU
% DAB SoOP TDOA Localization 
% =========================================================================
% Bu son adımda:
%   1. Tüm adımların sonuçlarını karşılaştırıyoruz
%   2. Kalman parametrelerini optimize ediyoruz
%   3. Monte Carlo simülasyonu yapıyoruz (500 tekrar - makale ile aynı)
%   4. Final raporu ve grafikleri üretiyoruz
% =========================================================================

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('  ADIM 8: Performans Analizi\n');
fprintf('========================================\n\n');

% =========================================================================
% BÖLÜM 1: VERİLERİ YÜKLE
% =========================================================================

load('geometry_data.mat');
load('dab_signal_data.mat');
load('tdoa_data.mat');
load('position_data.mat');
load('kalman_data.mat');

fprintf('✓ Tüm veriler yüklendi.\n\n');

% =========================================================================
% BÖLÜM 2: KALMAN PARAMETRELERİNİ OPTİMİZE ET
% =========================================================================
% Adım 7'de hata büyüdü çünkü:
% 1. sigma_meas çok büyük seçildi → filtre ölçüme güvenmedi
% 2. Sadece 20 nokta var → filtre yakınsamaya fırsat bulamadı
%
% Çözüm: sigma_meas'ı küçült, sigma_a'yı ayarla

fprintf('Kalman parametreleri optimize ediliyor...\n');

% En iyi parametreyi bulmak için tarama
sigma_a_vals    = [0.5, 1.0, 2.0, 5.0, 10.0];
sigma_meas_vals = [10, 30, 50, 80, 100];

best_rmse  = inf;
best_sa    = 0;
best_sm    = 0;
results_grid = zeros(length(sigma_a_vals), length(sigma_meas_vals));

for ia = 1:length(sigma_a_vals)
    for im = 1:length(sigma_meas_vals)
        sa = sigma_a_vals(ia);
        sm = sigma_meas_vals(im);

[~, ~, pos_err_tmp, ~] = run_kalman(UAV_estimated, UAV_true, N_pos, sa, sm);
rmse_k = sqrt(mean(pos_err_tmp.^2));

        results_grid(ia, im) = rmse_k;

        if rmse_k < best_rmse
            best_rmse = rmse_k;
            best_sa   = sa;
            best_sm   = sm;
        end
    end
end

fprintf('  En iyi σ_a    : %.1f m/adım²\n', best_sa);
fprintf('  En iyi σ_meas : %.1f m\n', best_sm);
fprintf('  En iyi RMSE   : %.2f m\n\n', best_rmse);

% Optimize edilmiş Kalman çalıştır
[UAV_filtered_opt, ~, pos_err_kalman_opt, K_gains_opt] = ...
    run_kalman(UAV_estimated, UAV_true, N_pos, best_sa, best_sm);

mean_kalman_opt = mean(pos_err_kalman_opt);
p95_kalman_opt  = prctile(pos_err_kalman_opt, 95);

fprintf('Optimize Kalman Sonuçları:\n');
fprintf('  Ortalama hata: %.2f m\n', mean_kalman_opt);
fprintf('  RMSE         : %.2f m\n', best_rmse);
fprintf('  P95          : %.2f m\n\n', p95_kalman_opt);

% =========================================================================
% BÖLÜM 3: MONTE CARLO SİMÜLASYONU (500 TEKRAR)
% =========================================================================
% Makale: 500 Monte Carlo trial, mean error 37.14m, P95 46.62m
% Biz de aynı yapıyı kuruyoruz.

fprintf('Monte Carlo simülasyonu (500 tekrar) başlıyor...\n');
fprintf('Bu birkaç dakika sürebilir...\n\n');

N_MC      = 500;
SNR_dB_mc = 15;

% Multipath parametreleri
path_delays_s   = round([0, 0.5, 1.2] * 1e-6 * Fs);
path_powers_lin = 10.^([0, -8, -15] / 10);

mc_errors_wnls   = zeros(N_MC, N_pos);
mc_errors_kalman = zeros(N_MC, N_pos);

for mc = 1:N_MC
    % Her tekrarda yeni gürültü realizasyonu
    TDOA_mc = zeros(N_pos, 4);

    % Ref RX TOA
    TOA_RefRX_mc = zeros(1, 4);
    for tx = 1:4
        delay_int = round(d_TX_RefRX(tx) / c * Fs);
        rx = make_rx_signal(dab_frame, delay_int, path_delays_s, ...
                            path_powers_lin, SNR_dB_mc, Fs);
        TOA_RefRX_mc(tx) = gcc_phat_local(rx, prs_symbol, N_null, N_prs, Fs);
    end

    % UAV TOA ve TDOA
    TOA_UAV_mc = zeros(N_pos, 4);
    for pos = 1:N_pos
        for tx = 1:4
            delay_int = round(d_TX_UAV(pos,tx) / c * Fs);
            rx = make_rx_signal(dab_frame, delay_int, path_delays_s, ...
                                path_powers_lin, SNR_dB_mc, Fs);
            TOA_UAV_mc(pos,tx) = gcc_phat_local(rx, prs_symbol, N_null, N_prs, Fs);
        end
        for tx = 1:4
            TDOA_mc(pos,tx) = (TOA_UAV_mc(pos,tx) - TOA_RefRX_mc(tx)) * c / Fs;
        end
    end

    % WNLS çözümü
    x0  = [mean(TX(:,1)); mean(TX(:,2)); 200];
    W   = eye(4);
    UAV_est_mc = zeros(N_pos, 3);
    for pos = 1:N_pos
        [x_est, ~, ~] = wnls_solver_local(TDOA_mc(pos,:)', TX, Ref_RX, x0, W);
        UAV_est_mc(pos,:) = x_est';
    end

    % Kalman filtresi
    [UAV_filt_mc, ~, ~, ~] = run_kalman(UAV_est_mc, UAV_true, N_pos, best_sa, best_sm);

    % Hatalar
    for pos = 1:N_pos
        mc_errors_wnls(mc,pos)   = norm(UAV_est_mc(pos,:)  - UAV_true(pos,:));
        mc_errors_kalman(mc,pos) = norm(UAV_filt_mc(pos,:) - UAV_true(pos,:));
    end

    if mod(mc, 100) == 0
        fprintf('  %d/500 tamamlandı...\n', mc);
    end
end

% Monte Carlo istatistikleri
all_errors_wnls   = mc_errors_wnls(:);
all_errors_kalman = mc_errors_kalman(:);

mc_mean_wnls   = mean(all_errors_wnls);
mc_mean_kalman = mean(all_errors_kalman);
mc_p95_wnls    = prctile(all_errors_wnls, 95);
mc_p95_kalman  = prctile(all_errors_kalman, 95);
mc_rmse_wnls   = sqrt(mean(all_errors_wnls.^2));
mc_rmse_kalman = sqrt(mean(all_errors_kalman.^2));

fprintf('\n=== MONTE CARLO SONUÇLARI (500 tekrar) ===\n');
fprintf('  %-25s  %-12s  %-12s\n', 'Metrik', 'WNLS', 'WNLS+Kalman');
fprintf('  %-25s  %-12s  %-12s\n', '-------------------------', '------------', '------------');
fprintf('  %-25s  %-12.2f  %-12.2f\n', 'Ortalama Hata [m]', mc_mean_wnls,   mc_mean_kalman);
fprintf('  %-25s  %-12.2f  %-12.2f\n', 'RMSE [m]',          mc_rmse_wnls,   mc_rmse_kalman);
fprintf('  %-25s  %-12.2f  %-12.2f\n', 'P95 Hata [m]',      mc_p95_wnls,    mc_p95_kalman);
fprintf('\n  Makale hedefi: Ort=37.14m, P95=46.62m\n\n');

% =========================================================================
% BÖLÜM 4: KAYDET
% =========================================================================

save('final_results.mat', ...
    'mc_errors_wnls', 'mc_errors_kalman', ...
    'mc_mean_wnls', 'mc_mean_kalman', ...
    'mc_rmse_wnls', 'mc_rmse_kalman', ...
    'mc_p95_wnls',  'mc_p95_kalman', ...
    'best_sa', 'best_sm', 'best_rmse', ...
    'UAV_filtered_opt', 'pos_err_kalman_opt');

fprintf('✓ Sonuçlar "final_results.mat" dosyasına kaydedildi.\n\n');

% =========================================================================
% BÖLÜM 5: FINAL GRAFİKLER
% =========================================================================

figure('Name', 'Adım 8: Final Analiz', 'NumberTitle', 'off', ...
       'Position', [30, 30, 1300, 900]);

% --- Panel 1: Monte Carlo Hata Histogramı ---
subplot(2, 3, 1);
hold on;
histogram(all_errors_wnls,   30, 'FaceColor', 'red',  'FaceAlpha', 0.6, ...
          'DisplayName', sprintf('WNLS (%.1fm)', mc_mean_wnls));
histogram(all_errors_kalman, 30, 'FaceColor', 'blue', 'FaceAlpha', 0.6, ...
          'DisplayName', sprintf('Kalman (%.1fm)', mc_mean_kalman));
xline(37.14, 'k--', 'LineWidth', 2, 'Label', 'Makale=37.14m');
xlabel('Konum Hatası [m]', 'FontSize', 10);
ylabel('Frekans', 'FontSize', 10);
title('Monte Carlo Hata Dağılımı (N=500)', 'FontWeight', 'bold');
legend('FontSize', 8, 'Location', 'northeast');
grid on;

% --- Panel 2: CDF Karşılaştırması ---
subplot(2, 3, 2);
err_w_s = sort(all_errors_wnls);
err_k_s = sort(all_errors_kalman);
n_total = length(err_w_s);
cdf_y   = (1:n_total) / n_total * 100;
plot(err_w_s, cdf_y, 'r-', 'LineWidth', 2.5, ...
     'DisplayName', sprintf('WNLS P95=%.0fm', mc_p95_wnls));
hold on;
plot(err_k_s, cdf_y, 'b-', 'LineWidth', 2.5, ...
     'DisplayName', sprintf('Kalman P95=%.0fm', mc_p95_kalman));
xline(37.14, 'k--', 'LineWidth', 1.5, 'Label', 'Makale=37.14m');
xline(46.62, 'k:',  'LineWidth', 1.5, 'Label', 'P95=46.62m');
yline(95, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xlabel('Konum Hatası [m]', 'FontSize', 10);
ylabel('Kümülatif Olasılık [%]', 'FontSize', 10);
title('CDF - Monte Carlo (N=500)', 'FontWeight', 'bold');
legend('FontSize', 8, 'Location', 'southeast');
grid on;
ylim([0, 100]);

% --- Panel 3: Optimize Kalman Yörüngesi ---
subplot(2, 3, 3);
hold on; grid on;
scatter(TX(:,1), TX(:,2), 100, 'bs', 'filled', 'HandleVisibility','off');
scatter(Ref_RX(1), Ref_RX(2), 100, 'g^', 'filled', 'HandleVisibility','off');
plot(UAV_true(:,1),          UAV_true(:,2),         'k-',  'LineWidth', 2.5, ...
     'DisplayName', 'Gerçek');
plot(UAV_estimated(:,1),     UAV_estimated(:,2),    'r--', 'LineWidth', 1.5, ...
     'DisplayName', sprintf('WNLS (%.0fm)', mean_err));
plot(UAV_filtered_opt(:,1),  UAV_filtered_opt(:,2), 'b-',  'LineWidth', 2, ...
     'DisplayName', sprintf('Kalman opt (%.0fm)', mean_kalman_opt));
scatter(UAV_true(:,1),         UAV_true(:,2),         40, 'ko', 'filled','HandleVisibility','off');
scatter(UAV_filtered_opt(:,1), UAV_filtered_opt(:,2), 40, 'b^', 'filled','HandleVisibility','off');
xlabel('X [m]', 'FontSize', 10);
ylabel('Y [m]', 'FontSize', 10);
title('Optimize Kalman Yörüngesi', 'FontWeight', 'bold');
legend('FontSize', 8, 'Location', 'northeast');
axis equal;
xlim([-200, 2200]);
ylim([-700, 2700]);

% --- Panel 4: Parametre Optimizasyon Haritası ---
subplot(2, 3, 4);
imagesc(sigma_meas_vals, sigma_a_vals, results_grid);
colorbar;
colormap(jet);
xlabel('σ_{meas} [m]', 'FontSize', 10);
ylabel('σ_a [m/adım²]', 'FontSize', 10);
title('Kalman Parametre Optimizasyonu (RMSE)', 'FontWeight', 'bold');
set(gca, 'YTick', 1:length(sigma_a_vals), 'YTickLabel', sigma_a_vals);
set(gca, 'XTick', 1:length(sigma_meas_vals), 'XTickLabel', sigma_meas_vals);
hold on;
[~, ia] = min(results_grid(:));
[r,cc]  = ind2sub(size(results_grid), ia);
plot(cc, r, 'w*', 'MarkerSize', 15, 'LineWidth', 2);

% --- Panel 5: Adım Adım Hata Özeti ---
subplot(2, 3, 5);
adim_labels = {'TDOA', 'WNLS', 'Kalman\nOpt'};
adim_errors = [mean(abs(TDOA_error_m(:))), mean_err, mean_kalman_opt];
b = bar(adim_errors, 'FaceColor', 'flat');
b.CData(1,:) = [1 0.5 0];
b.CData(2,:) = [0.8 0.2 0.2];
b.CData(3,:) = [0.2 0.4 0.8];
yline(37.14, 'k--', 'LineWidth', 2, 'Label', 'Makale=37.14m');
set(gca, 'XTickLabel', {'TDOA Hatası', 'WNLS', 'Kalman Opt'});
ylabel('Ortalama Hata [m]', 'FontSize', 10);
title('Pipeline Boyunca Hata', 'FontWeight', 'bold');
grid on;

% --- Panel 6: Özet Tablo ---
subplot(2, 3, 6);
axis off;
tablo = {
    'Metrik',           'WNLS',              'Kalman Opt',       'Makale';
    'Ort. Hata [m]',    sprintf('%.1f', mean_err),  sprintf('%.1f', mean_kalman_opt), '37.14';
    'RMSE [m]',         sprintf('%.1f', rmse_wnls),  sprintf('%.1f', best_rmse),      '-';
    'P95 [m]',          sprintf('%.1f', p95_wnls),   sprintf('%.1f', p95_kalman_opt), '46.62';
    'MC Ort. [m]',      sprintf('%.1f', mc_mean_wnls), sprintf('%.1f', mc_mean_kalman), '37.14';
    'MC P95 [m]',       sprintf('%.1f', mc_p95_wnls),  sprintf('%.1f', mc_p95_kalman),  '46.62';
};

t = uitable('Data', tablo(2:end,:), ...
            'ColumnName', tablo(1,:), ...
            'Units', 'normalized', ...
            'Position', [0.67, 0.02, 0.31, 0.28]);
t.ColumnWidth = {120, 80, 90, 70};

text(0.5, 0.95, 'Özet Sonuç Tablosu', 'HorizontalAlignment', 'center', ...
     'FontSize', 12, 'FontWeight', 'bold', 'Units', 'normalized');

sgtitle('ADIM 8: Final Performans Analizi - DAB SoOP TDOA', ...
        'FontSize', 14, 'FontWeight', 'bold');

% YENİ:
exportgraphics(gcf, 'step8_analysis.png', 'Resolution', 150);
fprintf('✓ Grafik "step8_analysis.png" kaydedildi.\n');

% =========================================================================
% ÖZET RAPOR
% =========================================================================
fprintf('\n');
fprintf('╔══════════════════════════════════════════╗\n');
fprintf('║     FİNAL PERFORMANS RAPORU              ║\n');
fprintf('║  DAB SoOP TDOA UAV Lokalizasyonu        ║\n');
fprintf('╠══════════════════════════════════════════╣\n');
fprintf('║  Tek Deneme (N_pos=20):                 ║\n');
fprintf('║    WNLS Ort. Hata    : %6.2f m         ║\n', mean_err);
fprintf('║    Kalman Opt. Hata  : %6.2f m         ║\n', mean_kalman_opt);
fprintf('╠══════════════════════════════════════════╣\n');
fprintf('║  Monte Carlo (N=500):                   ║\n');
fprintf('║    WNLS Ort. Hata    : %6.2f m         ║\n', mc_mean_wnls);
fprintf('║    Kalman Ort. Hata  : %6.2f m         ║\n', mc_mean_kalman);
fprintf('║    WNLS P95          : %6.2f m         ║\n', mc_p95_wnls);
fprintf('║    Kalman P95        : %6.2f m         ║\n', mc_p95_kalman);
fprintf('╠══════════════════════════════════════════╣\n');
fprintf('║  Makale Hedefleri:                      ║\n');
fprintf('║    Ort. Hata         :  37.14 m         ║\n');
fprintf('║    P95               :  46.62 m         ║\n');
fprintf('╚══════════════════════════════════════════╝\n\n');

fprintf('TÜM ADIMLAR TAMAMLANDI! 🎉\n\n');

% =========================================================================
% YEREL YARDIMCI FONKSİYONLAR
% =========================================================================

function [UAV_filt, x_filt_all, pos_errors, K_all] = run_kalman(UAV_est, UAV_tr, N, sa, sm)
    dt = 1;
    F  = [1 0 0 dt 0 0; 0 1 0 0 dt 0; 0 0 1 0 0 dt;
          0 0 0 1  0 0; 0 0 0 0 1  0; 0 0 0 0 0  1];
    H  = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];
    Q  = sa^2 * [dt^4/4 0 0 dt^3/2 0 0; 0 dt^4/4 0 0 dt^3/2 0;
                 0 0 dt^4/4 0 0 dt^3/2; dt^3/2 0 0 dt^2 0 0;
                 0 dt^3/2 0 0 dt^2 0;  0 0 dt^3/2 0 0 dt^2];
    R  = sm^2 * eye(3);

    x_k = [UAV_est(1,1); UAV_est(1,2); UAV_est(1,3); 0; 0; 0];
    P_k = diag([sm^2, sm^2, sm^2, 100^2, 100^2, 100^2]);

    UAV_filt   = zeros(N, 3);
    x_filt_all = zeros(6, N);
    pos_errors = zeros(N, 1);
    K_all      = zeros(6, 3, N);

    for k = 1:N
        x_pred = F * x_k;
        P_pred = F * P_k * F' + Q;
        z_k    = UAV_est(k,:)';
        y_k    = z_k - H * x_pred;
        S_k    = H * P_pred * H' + R;
        K_k    = P_pred * H' / S_k;
        x_k    = x_pred + K_k * y_k;
        P_k    = (eye(6) - K_k*H) * P_pred * (eye(6) - K_k*H)' + K_k*R*K_k';

        UAV_filt(k,:)    = x_k(1:3)';
        x_filt_all(:,k)  = x_k;
        pos_errors(k)    = norm(x_k(1:3)' - UAV_tr(k,:));
        K_all(:,:,k)     = K_k;
    end
end

function rx = make_rx_signal(frame, delay_int, path_delays, path_powers, snr_dB, Fs)
    if delay_int < length(frame)
        rx = [zeros(1, delay_int), frame(1:end-delay_int)];
    else
        rx = zeros(size(frame));
    end
    rx_mp = zeros(size(rx));
    for p = 1:length(path_delays)
        d = path_delays(p);
        if d == 0
            rx_mp = rx_mp + sqrt(path_powers(p)) * rx;
        else
            rx_mp = rx_mp + sqrt(path_powers(p)) * [zeros(1,d), rx(1:end-d)];
        end
    end
    sp    = mean(abs(rx_mp).^2);
    nl    = sp / 10^(snr_dB/10);
    rx    = rx_mp + sqrt(nl) * randn(size(rx_mp));
end

function toa = gcc_phat_local(rx, prs_ref, n_null, n_prs, Fs)
    s = n_null + 1;
    e = min(s + n_prs - 1, length(rx));
    seg = rx(s:e);
    ml  = min(length(seg), length(prs_ref));
    seg = seg(1:ml);
    ref = prs_ref(1:ml);
    N2  = 2^nextpow2(2*ml-1);
    G   = fft(seg,N2) .* conj(fft(ref,N2));
    G   = G ./ (abs(G) + 1e-10);
    c   = real(ifft(G,N2));
    [~,pk] = max(abs(c(1:floor(N2/2))));
    toa = n_null + pk - 1;
end

function [x_est, conv, n_iter] = wnls_solver_local(tdoa_vec, TX_pos, RefRX, x0, W)
    x_cur = x0(:); conv = false; n_iter = 0;
    for iter = 1:50
        n_iter = iter;
        r = zeros(4,1); J = zeros(4,3);
        for i = 1:4
            du = norm(x_cur - TX_pos(i,:)');
            dr = norm(RefRX' - TX_pos(i,:)');
            r(i) = tdoa_vec(i) - (du - dr);
            if du < 1e-6; du = 1e-6; end
            J(i,:) = (x_cur - TX_pos(i,:)') / du;
        end
        JtW = J'*W; JtWJ = JtW*J;
        if rcond(JtWJ) < 1e-12; break; end
        dx = JtWJ \ (JtW*r);
        x_cur = x_cur + dx;
        if norm(dx) < 1e-3; conv = true; break; end
    end
    x_est = x_cur;
end