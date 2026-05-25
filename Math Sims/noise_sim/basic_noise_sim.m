%% Parameters - fill from datasheets
  I_LED     = 20e-3;       % LED drive current [A]
  P_opt     = 10e-3;       % LED optical power [W] - from datasheet
  mu        = 0.5;         % plastic attenuation coeff [1/mm] - measure this
  d_nom     = 3;           % nominal wedge thickness [mm]
  R_resp    = 0.5;         % phototransistor responsivity [A/W]
  I_dark    = 10e-9;       % dark current [A] - datasheet
  R_load    = 10e3;        % load resistor [Ohm]
  V_ref     = 3.3;         % ADC reference [V]
  BW        = 100;         % measurement bandwidth [Hz]

  T  = 298;                % temperature [K]
  q  = 1.602e-19;
  kB = 1.38e-23;

  %% Signal chain
  I_opt    = P_opt * exp(-mu * d_nom);      % transmitted optical power [W]
  I_photo  = R_resp * I_opt;                % photocurrent [A]
  V_signal = I_photo * R_load;             % voltage at ADC [V]

  %% Noise sources [A]
  i_shot  = sqrt(2 * q * I_photo * BW);
  i_dark  = sqrt(2 * q * I_dark  * BW);
  i_R     = sqrt(4 * kB * T * BW / R_load);
  i_total = sqrt(i_shot^2 + i_dark^2 + i_R^2);

  %% ADC quantization [V]
  V_quant = V_ref / (2^12 * sqrt(12));

  %% Total noise voltage
  V_noise = sqrt((i_total * R_load)^2 + V_quant^2);

  %% Signal sensitivity [A/mm]
  dI_dd = R_resp * mu * I_opt;

  %% Noise-Equivalent Displacement
  NED_mm = V_noise / (dI_dd * R_load);   % [mm/√Hz]

  fprintf('Photocurrent:        %.2f µA\n',  I_photo*1e6);
  fprintf('Signal voltage:      %.2f mV\n',  V_signal*1e3);
  fprintf('Shot noise:          %.2f nA\n',  i_shot*1e9);
  fprintf('Johnson noise:       %.2f nA\n',  i_R*1e9);
  fprintf('ADC quant noise:     %.2f mV\n',  V_quant*1e3);
  fprintf('NED:                 %.4f mm/sqrt(Hz)\n', NED_mm);
  fprintf('NED @ %.0f Hz BW:    %.4f mm\n',  BW, NED_mm*sqrt(BW));