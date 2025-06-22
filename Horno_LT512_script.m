% ============ Trabajo Práctico Final de Sistemas de Control I ============
% Sistema de control de temperatura para un horno eléctrico de laboratorio
% =========================================================================
% Sistemas de control I - 2025
% Alumnas:
%  - Guyot Lourdes
%  - Mayorga Federica
% =========================================================================
% Antes de comenzar, limpiamos la ventana de comandos y cerramos todas las
% posibles figuras abiertas
clc ; close all

% === Primera Parte: Obtener la Función de Transferencia a Lazo Abierto ===
% Declaramos las variables del modelo matematico para el control de 
% temperatura del horno LT 5/12:
s = tf('s');

al = 0.13;                  % Alto  [mm]
an = 0.205;                 % Ancho [mm]
la = 0.17;                  % Largo [mm]
V = al*an*la;               % Volumen interno del Horno [m^3]

R = 9.77;                   % Resistencia termica                [°C/W]
d = 1.225;                  % Densidad del aire                  [Kg/m^3]
m = d*V;                    % Masa de aire contenida en el horno [Kg]
c = 1005;                   % Calor especifico del aire          [J/Kg K]
C = m*c;                    % Capacitancia termica               [J/K]
TEMP = 1473;                % Temperatura que buscamos           [K]

FdTLA = minreal(R/(C*R*s+1)) % Función de Transferencia a Lazo Abierto
%% ========================================================================
% ==== Segunda Parte: Obtener la Función de Transferencia lazo cerrado ====
% Dimensionamos el sensor basado en su sensibilidad y el amplificador
% necesario:
Gs_sensor = 0.011;
Gs_ampli = 90.9;
FdTLC = minreal(feedback(FdTLA, Gs_sensor*Gs_ampli))
step(TEMP*FdTLC); grid;
title('Step response closed loop'); 
ylabel('Temperature [K]');
% Sistema de orden 1 tiene un solo polo
pole(FdTLC)         % Respuesta por terminal de comandos
rlocus(FdTLC)       % Respuesta gráfica
%% ========================================================================
% ================== Tercera Parte: Error estado estable ==================
% Calculo del error y diseno de controlador PI:
Kp = evalfr(FdTLA, 0);
ess_porcentual = (1/(1+Kp))*100;
% Buscamos en el Lugar de Raices:
rlocus(FdTLA); sgrid
% Desarrollando la fórmula de controlador PI, observamos que 
% PI= Kp(s+1/Ti)/s, falta determinar los valores de Kp y Ti
% Entonces, hay que ajustar el cero del compensador para cancelar el polo 
% de mi Función de Transferencia: FdTLA2= PI(S) * FdTLA(s)
% Elegimos tiempo de integracion de tal forma que cancele el polo dominante
Ti = (-1/pole(FdTLA)) 
% Y hacemos uso de la forma general del controlador:
PI = (s+(1/Ti))/s ;
FdTLA2 = PI * FdTLA;
% Funcion de transferencia a lazo abierto con compensador:
rlocus(FdTLA2); sgrid 
% Punto de diseno:
s1 = -0.014;
% Evaluo la funcion de transferencia en el punto de trabajo
G1 = evalfr(FdTLA, s1); 
invK = abs(((s1+(1/Ti))/s1)*G1) % Condicion de modulo
%rlocus(FdTLA2)
Kp = 1/invK;
% Y obtenemos asi el valor final de nuestro controlador:
PI= Kp * PI;
FdTLApi = minreal( PI * FdTLA);
FdTLCpi = minreal(feedback(FdTLApi,1))
% Entonces, el error estado estable con controlador es:
Kp_pi = evalfr(FdTLApi, 0);
ep_pi = 1/(1+Kp_pi) 
% Consiguiendo la respuesta del escalon del sistema con controlador:
step(1475*FdTLCpi); grid
ylabel('Temperatura [K]')
% =========================================================================