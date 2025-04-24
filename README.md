# Simulación del robot ROMERIN en CoppeliaSim

Este repositorio contiene el proyecto de simulación del robot ROMERIN, desarrollado en el entorno CoppeliaSim (anteriormente conocido como V-REP), como parte de un Trabajo de Fin de Máster en Ingeniería Industrial (UPM).

## 📦 Contenido del repositorio

- `romerin.lua`: Script principal que gestiona la locomoción multipata del robot ROMERIN, el control de ventosas, las fases de movimiento y la compensación gravitatoria.
- `cam_control.lua`: Script independiente que controla la orientación angular de la cámara montada sobre el robot, mediante una interfaz gráfica basada en sliders (*pantilt system*).
- `romerin.ttt`: Archivo principal de la escena en CoppeliaSim. Contiene toda la estructura jerárquica del robot, los scripts integrados, sensores, actuadores y el sistema de cámara.

## 🧠 Funcionalidades principales

- Simulación de locomoción multipata por fases (subida – desplazamiento – bajada).
- Sistema de ventosas activo con sensores centrales y auxiliares.
- Compensación de gravedad basada en fuerzas aplicadas sobre el cuerpo.
- Simulación en planos inclinados de hasta 45° modificando la dirección del vector gravedad, manteniendo su módulo constante.
- Control visual mediante una cámara orientable montada sobre el robot.
- Interfaz de usuario con sliders para mover la cámara en tiempo real.

## 🛠️ Requisitos

- CoppeliaSim v4.4+ con soporte para motores Bullet.
- Plugin `simIK` habilitado.
- Scripts en Lua activados dentro de la escena.

## 👤 Autor

**Sergio Peñas Mayoral**  
Trabajo de Fin de Máster – UPM  
Tutor: Ernesto Gambao Galán  
Co-tutor: Carlos Prados Sesmero
