<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        En un robot real, tienes que hacer muchas cosas a ritmos diferentes.
        <br />
        - Leer el sensor láser (rápido: 20 veces por segundo). - Chequear la batería (lento: 1 vez
        por minuto). - Parpadear una luz LED (medio: 1 vez por segundo).
        <br /><br />
        Si usas un solo bucle `while`, el proceso más lento frenaría a los demás. Con los
        <strong>Timers</strong>, podemos tener múltiples relojes independientes en un solo nodo.
      </TextBlock>
    </div>

    <!-- 1. EL CONCEPTO DE MULTITASKING -->
    <div class="section-group">
      <SectionTitle>1. El Director de Orquesta</SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock>
            Imagina a ROS 2 (`rclpy.spin`) como un director de orquesta. Él no toca los
            instrumentos. Él solo espera.
            <br /><br />
            Cuando un Timer "suena" (vence su tiempo), levanta la mano y el director le dice: "¡Toca
            tu nota ahora!".
            <br /><br />
            Esto permite que un solo hilo maneje múltiples ritmos sin bloquearse.
          </TextBlock>
        </template>
        <template #right>
          <div class="timer-visual q-pa-md text-center">
            <!-- Visualización CSS de relojes -->
            <div class="clock c-fast">
              <div class="hand h-fast"></div>
              <div class="label">Láser (0.05s)</div>
            </div>
            <div class="clock c-slow">
              <div class="hand h-slow"></div>
              <div class="label">Batería (5s)</div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 2. CÓDIGO MULTI-TIMER -->
    <div class="section-group">
      <SectionTitle>2. Sistema de Soporte Vital</SectionTitle>
      <TextBlock>
        Vamos a programar el nodo <code>soporte_vital.py</code>. Tendrá dos responsabilidades: 1.
        Monitorear el Oxígeno (Crítico, rápido). 2. Reportar temperatura a la Tierra (Informativo,
        lento).
      </TextBlock>

      <CodeBlock
        title="soporte_vital.py"
        lang="python"
        content="import rclpy
from rclpy.node import Node

class SoporteVital(Node):
    def __init__(self):
        super().__init__('soporte_vital')

        # --- TIMER 1: RÁPIDO (0.5 segundos) ---
        # Verifica niveles de oxígeno constantemente
        self.timer_o2 = self.create_timer(0.5, self.chequear_oxigeno)

        # --- TIMER 2: LENTO (3.0 segundos) ---
        # Envía un reporte aburrido a Houston
        self.timer_temp = self.create_timer(3.0, self.reportar_clima)

        self.nivel_o2 = 100

    def chequear_oxigeno(self):
        self.nivel_o2 -= 1 # Simulamos consumo
        if self.nivel_o2 < 20:
            self.get_logger().error(f'¡ALERTA! O2 CRÍTICO: {self.nivel_o2}%')
        else:
            self.get_logger().info(f'O2 estable: {self.nivel_o2}%')

    def reportar_clima(self):
        self.get_logger().warn('--- Enviando reporte a Houston: 22°C ---')

def main(args=None):
    rclpy.init(args=args)
    nodo = SoporteVital()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()"
        :copyable="true"
      />
    </div>

    <!-- 3. LA TRAMPA DEL BLOQUEO -->
    <div class="section-group">
      <SectionTitle>3. ¿Qué pasa si una tarea tarda mucho?</SectionTitle>

      <AlertBlock type="danger" title="El Defecto de Single Threaded">
        Por defecto, los nodos de Python son "Single Threaded" (Un solo cerebro).
        <br />
        Si la función <code>reportar_clima</code> se queda dormida (`time.sleep(10)`), el director
        de orquesta se congela esperándola.
        <br />
        <strong
          >¡El chequeo de oxígeno dejará de ejecutarse durante 10 segundos y la tripulación
          morirá!</strong
        >
      </AlertBlock>

      <div class="q-my-md">
        <CodeBlock
          title="El Código Asesino (No hagas esto)"
          lang="python"
          content="import time

def reportar_clima(self):
    # Esto bloquea TODO el nodo. Los otros timers se pausan.
    time.sleep(10)
    self.get_logger().info('Reporte enviado')"
        />
      </div>

      <TextBlock>
        <strong>Solución:</strong> Para tareas pesadas, usamos
        <strong>MultiThreadedExecutor</strong>
        (lo veremos en el módulo avanzado) o dividimos la tarea en pasos pequeños.
        <br />
        <em>Regla: Los Callbacks de los Timers deben ser rápidos como un rayo.</em>
      </TextBlock>
    </div>

    <!-- 4. DEBUGGING: ROS2 DOCTOR -->
    <div class="section-group">
      <SectionTitle>4. Hz: Midiendo el Ritmo</SectionTitle>
      <TextBlock>
        ¿Cómo sabes si tu timer realmente está cumpliendo la frecuencia prometida? ROS 2 tiene una
        herramienta para medir los "latidos" de un tópico.
      </TextBlock>

      <CodeBlock
        title="Midiendo frecuencia"
        lang="bash"
        content="# Si publicaras los datos en un tópico...
ros2 topic hz /sensor_oxigeno

# Salida:
# average rate: 2.001 min: 0.499s max: 0.501s"
      />
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto Sincronizado</SectionTitle>
      <TextBlock>
        Modifica el código anterior: 1. Añade un 3er Timer ultra-rápido (0.1s) que imprima "Latido".
        2. Ejecuta el nodo. 3. Observa cómo los mensajes se mezclan en la terminal. 4. Introduce un
        <code>time.sleep(2)</code> artificial en el Timer de Oxígeno y observa cómo el "Latido" deja
        de salir. (¡Has comprobado el bloqueo!).
      </TextBlock>
    </div>
  </div>
</template>

<script setup lang="ts">
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

/* Visualización de Relojes */
.timer-visual {
  background: rgba(30, 41, 59, 0.5);
  border-radius: 12px;
  display: flex;
  justify-content: center;
  gap: 30px;
}

.clock {
  width: 80px;
  height: 80px;
  border: 4px solid #cbd5e1;
  border-radius: 50%;
  position: relative;
  background: #0f172a;
}
.hand {
  width: 4px;
  height: 35px;
  background: #f87171;
  position: absolute;
  top: 5px;
  left: 34px;
  transform-origin: bottom;
  border-radius: 4px;
}

/* Animaciones de rotación */
.h-fast {
  animation: spin 0.5s linear infinite;
  background: #38bdf8;
}
.h-slow {
  animation: spin 3s linear infinite;
  background: #fbbf24;
}

@keyframes spin {
  100% {
    transform: rotate(360deg);
  }
}

.label {
  position: absolute;
  bottom: -30px;
  width: 120px;
  left: -20px;
  text-align: center;
  font-size: 0.8rem;
  color: #94a3b8;
  font-family: 'Fira Code', monospace;
}
</style>
