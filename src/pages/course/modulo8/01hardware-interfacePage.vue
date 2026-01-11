<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      Bienvenido a las Grandes Ligas. Hasta ahora has controlado simulaciones.
      <strong>ros2_control</strong> es el framework que conecta tu software con el hierro real:
      motores, actuadores lineales y grippers. <br /><br />
      Es una arquitectura de tiempo real estricta. Aquí, un retraso de 1ms puede causar que un brazo
      robótico de 50kg oscile y destruya tu laboratorio.
    </TextBlock>

    <!-- ARCHITECTURE DIAGRAM (DIDACTIC) -->
    <div class="section-group">
      <SectionTitle>1. Arquitectura de Control (The Loop)</SectionTitle>

      <div class="control-arch-viz q-mt-md">
        <!-- USER LEVEL -->
        <div class="layer user">
          <div class="layer-label">User Land (Non-Realtime)</div>
          <div class="node-box">MoveIt / Nav2</div>
          <div class="topic-arrow">⬇ /joint_trajectory</div>
        </div>

        <!-- CONTROLLER MANAGER -->
        <div class="layer realtime">
          <div class="layer-label">Realtime Loop (1000Hz)</div>
          <div class="cm-box">
            <div class="cm-title">Controller Manager</div>
            <div class="controllers-container">
              <div class="controller">DiffDrive Controller</div>
              <div class="controller">JointState Broadcaster</div>
            </div>
            <div class="interface-arrow">⬇ Read/Write Interfaces</div>
            <div class="resource-manager">
              <div class="rm-title">Resource Manager</div>
              <div class="hardware-box">
                <q-icon name="memory" color="orange" />
                Hardware Interface (C++)
              </div>
            </div>
          </div>
        </div>

        <!-- HARDWARE LEVEL -->
        <div class="layer hardware">
          <div class="layer-label">Physical World</div>
          <div class="device-box"><q-icon name="settings_input_component" /> Motors / Encoders</div>
        </div>
      </div>
    </div>

    <!-- WRITING HW INTERFACE -->
    <div class="section-group">
      <SectionTitle>2. Escribiendo un Hardware Interface (C++)</SectionTitle>
      <TextBlock>
        Para controlar tu robot custom, debes heredar de
        <code>hardware_interface::SystemInterface</code>. Este código corre dentro del loop de
        tiempo real.
      </TextBlock>

      <AlertBlock type="danger" title="Reglas Estrictas de Realtime">
        1. <strong>Sin `malloc`:</strong> No asigne memoria dinámica (use `std::vector::reserve`).
        <br />
        2. <strong>Sin `printf`:</strong> I/O bloquea. Use RT-Loggers.
        <br />
        3. <strong>Sin Exceptions:</strong> No puede crashear.
      </AlertBlock>

      <div class="code-implementation q-mt-md">
        <CodeBlock
          title="my_robot_hardware.cpp"
          lang="cpp"
          content='#include "hardware_interface/system_interface.hpp"

class MyRobotHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const HardwareInfo & info) override
  {
    if (info.hardware_parameters.count("baud_rate") == 0) {
      // Error fatal en configuración
      return CallbackReturn::ERROR;
    }

    // Reservar memoria para command/state vectors
    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    return CallbackReturn::SUCCESS;
  }

  // Se llama a 1000Hz (muy rápido!)
  return_type read(const Time & time, const Duration & period) override
  {
    // Leer encoders vía Serial/EtherCAT
    // IMPORTANTE: Esta funcion debe retornar en <1ms
    for (uint i = 0; i < hw_positions_.size(); i++) {
        hw_positions_[i] = serial_driver_->get_encoder(i);
    }
    return return_type::OK;
  }

  return_type write(const Time & time, const Duration & period) override
  {
    // Escribir comandos PWM a motores
    for (uint i = 0; i < hw_commands_.size(); i++) {
        serial_driver_->send_pwm(i, hw_commands_[i]);
    }
    return return_type::OK;
  }

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
};'
          :copyable="true"
        />
      </div>
    </div>

    <!-- ROS2_CONTROL TAGS -->
    <div class="section-group">
      <SectionTitle>3. URDF Integration</SectionTitle>
      <TextBlock>
        Debes decirle a `ros2_control` qué recursos posee tu robot en el URDF.
      </TextBlock>

      <CodeBlock
        title="robot.urdf.xacro"
        lang="xml"
        content='<ros2_control name="MyRobot" type="system">
  <hardware>
    <plugin>my_robot_package/MyRobotHardware</plugin>
    <param name="baud_rate">115200</param>
    <param name="port">/dev/ttyUSB0</param>
  </hardware>

  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10</param>
      <param name="max">10</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>'
        :copyable="true"
      />
    </div>

    <!-- DOCTORAL CHALLENGE -->
    <div class="section-group">
      <SectionTitle>4. Doctor''s Challenge: Integration Nightmare</SectionTitle>

      <div class="challenge-card">
        <div class="challenge-title">⏱️ El Jitter Asesino</div>
        <div class="challenge-desc">
          Tu robot tiembla violentamente cada 5 segundos. Has revisado los PIDs y están bien. Al
          hacer profiling, ves que tu función <code>read()</code> tarda a veces 2ms en lugar de
          0.1ms.
          <br />
          ¿Cuál es el culpable más probable en tu código C++?
        </div>

        <div class="options-grid">
          <div class="option wrong">
            <div class="opt-head">A. Usar double en lugar de float</div>
            <div class="opt-body">No, las CPUs modernas procesan double igual de rápido.</div>
          </div>
          <div class="option correct">
            <div class="opt-head">B. cout << "Encoder: " << val << endl;</div>
            <div class="opt-body">
              ¡CORRECTO! Imprimir a consola bloquea el hilo y rompe el determinismo temporal. NUNCA
              uses <code>std::cout</code> en realtime loops.
            </div>
          </div>
          <div class="option wrong">
            <div class="opt-head">C. Leer un array de 10 elementos</div>
            <div class="opt-body">Esto es trivial para la CPU (nanosegundos).</div>
          </div>
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>SystemInterface</code>
          <span>Clase base para robots completos (escritura/lectura)</span>
        </div>
        <div class="summary-item">
          <code>Controller Manager</code>
          <span>Nodo que orquesta el ciclo de control y carga controladores</span>
        </div>
        <div class="summary-item">
          <code>Resource Manager</code>
          <span>Abstracción que evita conflictos de acceso al hardware</span>
        </div>
        <div class="summary-item">
          <code>Realtime Safety</code>
          <span>Restricciones de memoria e I/O estrictas</span>
        </div>
      </div>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

/* ARCHITECTURE VIZ */
.control-arch-viz {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
  background: rgba(15, 23, 42, 0.8);
  padding: 2rem;
  border-radius: 12px;
}

.layer {
  width: 100%;
  max-width: 500px;
  padding: 1.5rem;
  border-radius: 8px;
  border: 1px dashed rgba(255, 255, 255, 0.2);
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
}

.layer-label {
  position: absolute;
  top: -12px;
  left: 10px;
  background: #0f172a;
  padding: 0 10px;
  font-size: 0.8rem;
  color: #94a3b8;
  font-weight: 700;
}

.layer.user {
  border-color: #64748b;
}
.layer.realtime {
  border-color: #ef4444;
  background: rgba(239, 68, 68, 0.1);
}
.layer.hardware {
  border-color: #f59e0b;
}

.node-box {
  background: #334155;
  color: #fff;
  padding: 0.5rem 2rem;
  border-radius: 4px;
}

.cm-box {
  background: #991b1b;
  width: 100%;
  padding: 1rem;
  border-radius: 6px;
  display: flex;
  flex-direction: column;
  align-items: center;
}

.cm-title {
  color: #fecaca;
  font-weight: 700;
  margin-bottom: 1rem;
}

.controllers-container {
  display: flex;
  gap: 1rem;
  margin-bottom: 1rem;
}

.controller {
  background: #ef4444;
  color: #fff;
  padding: 0.25rem 0.5rem;
  font-size: 0.8rem;
  border-radius: 4px;
}

.resource-manager {
  background: #7f1d1d;
  width: 90%;
  padding: 0.5rem;
  border-radius: 4px;
  text-align: center;
}

.rm-title {
  font-size: 0.75rem;
  color: #fca5a5;
  margin-bottom: 0.5rem;
}

.hardware-box {
  background: #f97316;
  color: #fff;
  padding: 0.5rem;
  border-radius: 4px;
  font-weight: 700;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
}

.topic-arrow,
.interface-arrow {
  color: #94a3b8;
  font-family: monospace;
  margin: 0.5rem 0;
  font-size: 0.8rem;
}

.device-box {
  color: #f59e0b;
  font-size: 1.2rem;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

/* CHALLENGE */
.challenge-card {
  background: rgba(30, 41, 59, 0.8);
  border-radius: 16px;
  padding: 2rem;
  border-left: 5px solid #f97316;
}

.challenge-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #fdba74;
  margin-bottom: 0.5rem;
}
.challenge-desc {
  color: #cbd5e1;
  margin-bottom: 1.5rem;
}

.options-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1rem;
}

.option {
  background: rgba(0, 0, 0, 0.3);
  padding: 1rem;
  border-radius: 8px;
  cursor: pointer;
}

.option.correct:active {
  border: 1px solid #22c55e;
}
.option.wrong:active {
  border: 1px solid #ef4444;
}

.opt-head {
  font-weight: 700;
  color: #cbd5e1;
  margin-bottom: 0.5rem;
}
.opt-body {
  font-size: 0.85rem;
  color: #94a3b8;
}

/* SUMMARY & RESPONSIVE */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: rgba(15, 23, 42, 0.6);
  padding: 1rem;
  border-radius: 8px;
  display: flex;
  flex-direction: column;
}

.summary-item code {
  color: #fdba74;
  font-family: monospace;
}
.summary-item span {
  font-size: 0.85rem;
  color: #94a3b8;
}

@media (max-width: 1024px) {
  .options-grid {
    grid-template-columns: 1fr;
  }
}
</style>
