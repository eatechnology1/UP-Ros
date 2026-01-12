<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      Python es el lenguaje de facto para ROS 2. No por ser "fácil", sino porque permite
      <strong>iterar rápido</strong> sin compilar. En robótica, donde un bug puede costar $10,000 en
      hardware roto, la velocidad de desarrollo es crítica. <br /><br />
      Esta lección te enseña a escribir scripts Python que funcionen como
      <strong>procesos autónomos</strong>, no como código web. Aprenderás el shebang, permisos de
      ejecución, y las mejores prácticas para ROS 2.
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué Python y no C++?">
      <strong>Python:</strong> Prototipado rápido, debugging fácil, ideal para lógica de alto nivel
      (navegación, IA, planificación).
      <br />
      <strong>C++:</strong> Máximo rendimiento, control de memoria, ideal para drivers de sensores y
      control en tiempo real. <br /><br />
      En ROS 2, usarás <strong>ambos</strong>. Python para el cerebro, C++ para los reflejos.
    </AlertBlock>

    <!-- ANATOMÍA -->
    <div class="section-group">
      <SectionTitle>1. Anatomía de un Script Python para ROS 2</SectionTitle>
      <TextBlock>
        Un script de robótica NO es un script web. No tiene "main.py" que corre una vez y termina.
        Es un <strong>proceso continuo</strong> que escucha sensores, toma decisiones, y envía
        comandos en un loop infinito.
      </TextBlock>

      <div class="code-demo q-mt-md">
        <div class="code-header">
          <div class="code-dots">
            <div class="dot red"></div>
            <div class="dot yellow"></div>
            <div class="dot green"></div>
          </div>
          <div class="code-title">robot_brain.py</div>
        </div>
        <div class="code-body">
          <div class="code-line">
            <span class="line-num">1</span>
            <span class="token-shebang">#!/usr/bin/env python3</span>
            <div class="line-tooltip">🔧 Shebang: Le dice a Linux qué intérprete usar</div>
          </div>
          <div class="code-line">
            <span class="line-num">2</span>
            <span class="token-comment"># -*- coding: utf-8 -*-</span>
          </div>
          <div class="code-line empty">
            <span class="line-num">3</span>
          </div>
          <div class="code-line">
            <span class="line-num">4</span>
            <span class="token-keyword">import</span> rclpy
          </div>
          <div class="code-line">
            <span class="line-num">5</span>
            <span class="token-keyword">from</span> rclpy.node
            <span class="token-keyword">import</span> Node
            <div class="line-tooltip">📦 Imports: Librerías necesarias</div>
          </div>
          <div class="code-line empty">
            <span class="line-num">6</span>
          </div>
          <div class="code-line">
            <span class="line-num">7</span>
            <span class="token-keyword">class</span>
            <span class="token-class">RobotNode</span>(Node):
          </div>
          <div class="code-line">
            <span class="line-num">8</span>
            <span class="indent"></span><span class="token-keyword">def</span>
            <span class="token-func">__init__</span>(self):
          </div>
          <div class="code-line">
            <span class="line-num">9</span>
            <span class="indent"></span><span class="indent"></span>super().__init__(<span
              class="token-string"
              >'robot_brain'</span
            >)
          </div>
          <div class="code-line">
            <span class="line-num">10</span>
            <span class="indent"></span><span class="indent"></span>self.get_logger().info(<span
              class="token-string"
              >'Robot iniciado'</span
            >)
          </div>
          <div class="code-line empty">
            <span class="line-num">11</span>
          </div>
          <div class="code-line highlight">
            <span class="line-num">12</span>
            <span class="token-keyword">def</span> <span class="token-func">main</span>():
            <div class="line-tooltip">🎯 Entry point: Función principal</div>
          </div>
          <div class="code-line">
            <span class="line-num">13</span>
            <span class="indent"></span>rclpy.init()
          </div>
          <div class="code-line">
            <span class="line-num">14</span>
            <span class="indent"></span>node = RobotNode()
          </div>
          <div class="code-line">
            <span class="line-num">15</span>
            <span class="indent"></span>rclpy.spin(node)
          </div>
          <div class="code-line empty">
            <span class="line-num">16</span>
          </div>
          <div class="code-line highlight">
            <span class="line-num">17</span>
            <span class="token-keyword">if</span> __name__ ==
            <span class="token-string">'__main__'</span>:
            <div class="line-tooltip">🔒 Guardia: Solo ejecuta si se llama directamente</div>
          </div>
          <div class="code-line">
            <span class="line-num">18</span>
            <span class="indent"></span>main()
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="success" title="Estructura Estándar">
          <ol class="q-pl-md">
            <li><strong>Shebang:</strong> Primera línea, siempre</li>
            <li><strong>Imports:</strong> Librerías necesarias</li>
            <li><strong>Clases/Funciones:</strong> Tu lógica de negocio</li>
            <li><strong>main():</strong> Punto de entrada</li>
            <li><strong>if __name__ == '__main__':</strong> Guardia de ejecución</li>
          </ol>
        </AlertBlock>
      </div>
    </div>

    <!-- SHEBANG -->
    <div class="section-group">
      <SectionTitle>2. El Shebang: La Línea Mágica</SectionTitle>
      <TextBlock>
        En Linux, la extensión <code>.py</code> es solo decorativa. El sistema operativo lee la
        <strong>primera línea</strong> del archivo (shebang <code>#!</code>) para saber qué
        intérprete usar.
      </TextBlock>

      <div class="shebang-comparison q-mt-md">
        <div class="comparison-item correct">
          <div class="comparison-header">
            <q-icon name="check_circle" color="green-4" size="lg" />
            <span>✅ Forma Correcta (Portable)</span>
          </div>
          <div class="comparison-content">
            <CodeBlock lang="python" content="#!/usr/bin/env python3" :copyable="true" />
            <div class="comparison-note">
              <strong>¿Por qué?</strong> <code>env</code> busca <code>python3</code> en tu PATH.
              Funciona en cualquier sistema, incluso con entornos virtuales.
            </div>
          </div>
        </div>

        <div class="comparison-item wrong">
          <div class="comparison-header">
            <q-icon name="cancel" color="red-4" size="lg" />
            <span>❌ Forma Rígida (Evitar)</span>
          </div>
          <div class="comparison-content">
            <CodeBlock lang="python" content="#!/usr/bin/python3" />
            <div class="comparison-note">
              <strong>Problema:</strong> Ruta absoluta. Si Python está en otro lugar (ej:
              <code>/usr/local/bin/python3</code>), el script fallará.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- CHMOD -->
    <div class="section-group">
      <SectionTitle>3. Permisos de Ejecución: chmod +x</SectionTitle>
      <TextBlock>
        Por seguridad, Linux crea archivos como <strong>"solo lectura/escritura"</strong>. Nadie
        puede ejecutarlos por defecto. Debes otorgar permisos de ejecución con <code>chmod</code>.
      </TextBlock>

      <div class="chmod-demo q-mt-md">
        <div class="chmod-before">
          <div class="file-icon inactive">
            <q-icon name="description" size="4rem" />
          </div>
          <div class="file-name">script.py</div>
          <div class="file-perms">-rw-r--r--</div>
          <div class="file-status error">No ejecutable</div>
        </div>

        <div class="chmod-action">
          <q-icon name="bolt" color="yellow-6" size="3rem" />
          <CodeBlock
            :hide-header="true"
            lang="bash"
            content="chmod +x script.py"
            :copyable="true"
          />
        </div>

        <div class="chmod-after">
          <div class="file-icon active">
            <q-icon name="terminal" size="4rem" />
          </div>
          <div class="file-name">script.py</div>
          <div class="file-perms">-rwxr-xr-x</div>
          <div class="file-status success">✅ Ejecutable</div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="info" title="¿Qué hace chmod +x?">
          Agrega el permiso de <strong>ejecución (x)</strong> para el propietario, grupo y otros.
          Ahora puedes ejecutar el script con <code>./script.py</code> en lugar de
          <code>python3 script.py</code>.
        </AlertBlock>
      </div>
    </div>

    <!-- FORMAS DE EJECUTAR -->
    <div class="section-group">
      <SectionTitle>4. Tres Formas de Ejecutar un Script</SectionTitle>

      <div class="execution-methods">
        <div class="method-card">
          <div class="method-number">1</div>
          <div class="method-content">
            <div class="method-title">Método Manual</div>
            <CodeBlock lang="bash" content="python3 script.py" :copyable="true" />
            <div class="method-desc">
              <strong>Ventajas:</strong> No requiere shebang ni chmod
              <br />
              <strong>Desventajas:</strong> Debes especificar el intérprete manualmente
              <br />
              <strong>Uso:</strong> Pruebas rápidas, debugging
            </div>
          </div>
        </div>

        <div class="method-card">
          <div class="method-number">2</div>
          <div class="method-content">
            <div class="method-title">Método Directo (Linux)</div>
            <CodeBlock lang="bash" content="./script.py" :copyable="true" />
            <div class="method-desc">
              <strong>Ventajas:</strong> Más limpio, estilo Unix
              <br />
              <strong>Requisitos:</strong> Shebang + chmod +x
              <br />
              <strong>Uso:</strong> Scripts standalone, herramientas CLI
            </div>
          </div>
        </div>

        <div class="method-card ros">
          <div class="method-number">3</div>
          <div class="method-content">
            <div class="method-title">Método ROS 2 (Profesional)</div>
            <CodeBlock lang="bash" content="ros2 run mi_paquete mi_nodo" :copyable="true" />
            <div class="method-desc">
              <strong>Ventajas:</strong> ROS encuentra el ejecutable automáticamente
              <br />
              <strong>Requisitos:</strong> Paquete instalado con colcon
              <br />
              <strong>Uso:</strong> Producción, nodos ROS 2
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="warning" title="El Misterio del ./">
          ¿Por qué <code>./script.py</code> y no solo <code>script.py</code>? <br /><br />
          Por <strong>seguridad</strong>. Linux no ejecuta archivos del directorio actual por
          defecto (a diferencia de Windows). El <code>./</code> dice explícitamente: "ejecuta este
          archivo de AQUÍ".
        </AlertBlock>
      </div>
    </div>

    <!-- PYTHON PATH -->
    <div class="section-group">
      <SectionTitle>5. Python Path y Imports</SectionTitle>
      <TextBlock>
        Cuando haces <code>import mi_modulo</code>, Python busca en una lista de directorios llamada
        <strong>PYTHONPATH</strong>. Si tu módulo no está ahí, obtendrás
        <code>ModuleNotFoundError</code>.
      </TextBlock>

      <div class="path-demo q-mt-md">
        <div class="path-header">
          <q-icon name="search" color="blue-4" size="md" />
          <span>¿Dónde busca Python?</span>
        </div>
        <div class="path-list">
          <div class="path-item">
            <div class="path-num">1</div>
            <div class="path-desc">Directorio actual del script</div>
          </div>
          <div class="path-item">
            <div class="path-num">2</div>
            <div class="path-desc">Directorios en <code>PYTHONPATH</code></div>
          </div>
          <div class="path-item">
            <div class="path-num">3</div>
            <div class="path-desc">Librerías estándar (<code>/usr/lib/python3.x</code>)</div>
          </div>
          <div class="path-item">
            <div class="path-num">4</div>
            <div class="path-desc">Paquetes instalados (<code>site-packages</code>)</div>
          </div>
          <div class="path-item">
            <div class="path-num">5</div>
            <div class="path-desc">ROS 2 packages (si hiciste <code>source setup.bash</code>)</div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <CodeBlock
          title="Ver tu PYTHONPATH actual"
          lang="python"
          content="import sys
print('\n'.join(sys.path))"
          :copyable="true"
        />
      </div>
    </div>

    <!-- DEBUGGING -->
    <div class="section-group">
      <SectionTitle>6. Debugging: El Arte de Cazar Bugs</SectionTitle>

      <div class="debug-techniques">
        <div class="debug-card">
          <div class="debug-header">
            <q-icon name="bug_report" color="red-4" size="lg" />
            <span>Print Debugging (Básico)</span>
          </div>
          <div class="debug-content">
            <CodeBlock
              lang="python"
              content="def process_data(value):
    print(f'DEBUG: value = {value}')  # Debugging
    result = value * 2
    print(f'DEBUG: result = {result}')
    return result"
              :copyable="true"
            />
            <div class="debug-note">
              <strong>Ventaja:</strong> Simple, funciona siempre
              <br />
              <strong>Desventaja:</strong> Ensucian el código, difícil de limpiar
            </div>
          </div>
        </div>

        <div class="debug-card">
          <div class="debug-header">
            <q-icon name="terminal" color="blue-4" size="lg" />
            <span>Logging (Profesional)</span>
          </div>
          <div class="debug-content">
            <CodeBlock
              lang="python"
              content="import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def process_data(value):
    logger.debug(f'Processing value: {value}')
    result = value * 2
    logger.info(f'Result: {result}')
    return result"
              :copyable="true"
            />
            <div class="debug-note">
              <strong>Ventaja:</strong> Niveles (DEBUG, INFO, WARNING, ERROR), fácil de desactivar
              <br />
              <strong>Uso en ROS 2:</strong> <code>self.get_logger().info()</code>
            </div>
          </div>
        </div>

        <div class="debug-card">
          <div class="debug-header">
            <q-icon name="code" color="green-4" size="lg" />
            <span>Debugger (Avanzado)</span>
          </div>
          <div class="debug-content">
            <CodeBlock
              lang="python"
              content="import pdb

def process_data(value):
    pdb.set_trace()  # Breakpoint
    result = value * 2
    return result

# Ejecutar: python3 script.py
# Comandos: n (next), c (continue), p variable (print)"
              :copyable="true"
            />
            <div class="debug-note">
              <strong>Ventaja:</strong> Inspección interactiva, paso a paso
              <br />
              <strong>Alternativa:</strong> VS Code debugger (más visual)
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>Errores Comunes y Soluciones</SectionTitle>

      <q-expansion-item
        icon="error"
        label="bash: ./script.py: Permission denied"
        header-class="error-header"
      >
        <div class="error-content">
          <strong>Causa:</strong> El archivo no tiene permisos de ejecución. <br /><br />
          <strong>Solución:</strong>
          <CodeBlock lang="bash" content="chmod +x script.py" :copyable="true" />
        </div>
      </q-expansion-item>

      <q-expansion-item
        icon="error"
        label="ModuleNotFoundError: No module named 'rclpy'"
        header-class="error-header"
        class="q-mt-sm"
      >
        <div class="error-content">
          <strong>Causa:</strong> No hiciste <code>source</code> del setup.bash de ROS 2.
          <br /><br />
          <strong>Solución:</strong>
          <CodeBlock
            lang="bash"
            content="source /opt/ros/humble/setup.bash
# O agrega esto a ~/.bashrc"
            :copyable="true"
          />
        </div>
      </q-expansion-item>

      <q-expansion-item
        icon="error"
        label="SyntaxError: invalid syntax (shebang visible en output)"
        header-class="error-header"
        class="q-mt-sm"
      >
        <div class="error-content">
          <strong>Causa:</strong> Ejecutaste el script con <code>python script.py</code> en lugar de
          <code>python3 script.py</code>. <br /><br />
          <strong>Solución:</strong> Usa siempre <code>python3</code> o configura un alias:
          <CodeBlock lang="bash" content="alias python=python3" :copyable="true" />
        </div>
      </q-expansion-item>
    </div>

    <!-- EJEMPLO PRÁCTICO -->
    <div class="section-group">
      <SectionTitle>7. Ejemplo Práctico: Script ROS 2 Completo</SectionTitle>
      <TextBlock>
        Este es un nodo ROS 2 mínimo pero funcional que publica mensajes cada segundo:
      </TextBlock>

      <CodeBlock
        title="talker_node.py"
        lang="python"
        content="#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Talker node started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main():
    rclpy.init()
    node = TalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()"
        :copyable="true"
      />

      <div class="q-mt-md">
        <AlertBlock type="success" title="Cómo ejecutarlo">
          <CodeBlock
            lang="bash"
            content="# 1. Dar permisos
chmod +x talker_node.py

# 2. Ejecutar directamente
./talker_node.py

# 3. En otra terminal, escuchar
ros2 topic echo /chatter"
            :copyable="true"
          />
        </AlertBlock>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Python Scripts en Linux"
            frameborder="0"
            allow="
              accelerometer;
              autoplay;
              clipboard-write;
              encrypted-media;
              gyroscope;
              picture-in-picture;
            "
            allowfullscreen
          ></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" color="blue-4" size="sm" />
          Video En progreso
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen de Comandos Esenciales</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>chmod +x script.py</code>
          <span>Dar permisos de ejecución</span>
        </div>
        <div class="summary-item">
          <code>./script.py</code>
          <span>Ejecutar script directamente</span>
        </div>
        <div class="summary-item">
          <code>python3 script.py</code>
          <span>Ejecutar con intérprete manual</span>
        </div>
        <div class="summary-item">
          <code>which python3</code>
          <span>Ver ruta del intérprete Python</span>
        </div>
        <div class="summary-item">
          <code>python3 -m pdb script.py</code>
          <span>Ejecutar con debugger</span>
        </div>
        <div class="summary-item">
          <code>#!/usr/bin/env python3</code>
          <span>Shebang correcto (portable)</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Script Profesional" class="q-mt-lg">
        ✅ Shebang en la primera línea
        <br />
        ✅ Imports organizados (estándar → terceros → locales)
        <br />
        ✅ Funciones/clases bien documentadas
        <br />
        ✅ <code>if __name__ == '__main__':</code> al final
        <br />
        ✅ Manejo de excepciones (try/except)
        <br />
        ✅ Logging en lugar de prints
        <br />
        ✅ Permisos de ejecución (<code>chmod +x</code>)
      </AlertBlock>
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

/* CODE DEMO */
.code-demo {
  background: #1e1e1e;
  border: 1px solid #444;
  border-radius: 12px;
  overflow: hidden;
  font-family: 'Fira Code', monospace;
}

.code-header {
  background: #2d2d2d;
  padding: 0.75rem 1.5rem;
  display: flex;
  align-items: center;
  gap: 1rem;
  border-bottom: 1px solid #444;
}

.code-dots {
  display: flex;
  gap: 0.5rem;
}

.dot {
  width: 12px;
  height: 12px;
  border-radius: 50%;
}

.dot.red {
  background: #ff5f56;
}
.dot.yellow {
  background: #ffbd2e;
}
.dot.green {
  background: #27c93f;
}

.code-title {
  color: #94a3b8;
  font-size: 0.9rem;
}

.code-body {
  padding: 1.5rem;
}

.code-line {
  display: flex;
  align-items: center;
  padding: 0.25rem 0;
  position: relative;
  transition: background 0.2s;
}

.code-line:hover {
  background: rgba(255, 255, 255, 0.05);
}

.code-line.empty {
  min-height: 1.5rem;
}

.code-line.highlight {
  background: rgba(56, 189, 248, 0.1);
  border-left: 3px solid #38bdf8;
  padding-left: 0.5rem;
}

.line-num {
  color: #64748b;
  min-width: 3ch;
  margin-right: 1.5rem;
  text-align: right;
  user-select: none;
}

.token-shebang {
  color: #f472b6;
  font-style: italic;
}

.token-comment {
  color: #6b7280;
  font-style: italic;
}

.token-keyword {
  color: #c084fc;
  font-weight: 700;
}

.token-class {
  color: #fbbf24;
}

.token-func {
  color: #60a5fa;
}

.token-string {
  color: #4ade80;
}

.indent {
  display: inline-block;
  width: 2ch;
}

.line-tooltip {
  position: absolute;
  left: 100%;
  margin-left: 1rem;
  background: rgba(59, 130, 246, 0.9);
  color: white;
  padding: 0.25rem 0.75rem;
  border-radius: 4px;
  font-size: 0.75rem;
  white-space: nowrap;
  opacity: 0;
  pointer-events: none;
  transition: opacity 0.2s;
}

.code-line:hover .line-tooltip {
  opacity: 1;
}

/* SHEBANG COMPARISON */
.shebang-comparison {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.comparison-item {
  border-radius: 16px;
  overflow: hidden;
}

.comparison-item.correct {
  border: 2px solid rgba(34, 197, 94, 0.5);
}

.comparison-item.wrong {
  border: 2px solid rgba(239, 68, 68, 0.5);
}

.comparison-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  font-weight: 700;
  font-size: 1.05rem;
}

.comparison-item.correct .comparison-header {
  background: rgba(20, 83, 45, 0.3);
  color: #86efac;
}

.comparison-item.wrong .comparison-header {
  background: rgba(127, 29, 29, 0.3);
  color: #fca5a5;
}

.comparison-content {
  padding: 1.5rem;
  background: rgba(15, 23, 42, 0.6);
}

.comparison-note {
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  font-size: 0.9rem;
  color: #cbd5e1;
}

/* CHMOD DEMO */
.chmod-demo {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
  padding: 2rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
}

.chmod-before,
.chmod-after {
  text-align: center;
}

.file-icon {
  margin-bottom: 1rem;
}

.file-icon.inactive {
  color: #64748b;
}

.file-icon.active {
  color: #22c55e;
}

.file-name {
  font-family: 'Fira Code', monospace;
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}

.file-perms {
  font-family: 'Fira Code', monospace;
  color: #94a3b8;
  margin-bottom: 0.75rem;
}

.file-status {
  padding: 0.5rem 1rem;
  border-radius: 6px;
  font-size: 0.85rem;
  font-weight: 700;
}

.file-status.error {
  background: rgba(239, 68, 68, 0.1);
  color: #fca5a5;
}

.file-status.success {
  background: rgba(34, 197, 94, 0.1);
  color: #86efac;
}

.chmod-action {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
}

/* EXECUTION METHODS */
.execution-methods {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.method-card {
  display: flex;
  gap: 1.5rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.method-card.ros {
  border: 2px solid rgba(56, 189, 248, 0.5);
  background: rgba(56, 189, 248, 0.05);
}

.method-number {
  min-width: 3rem;
  height: 3rem;
  background: linear-gradient(135deg, #8b5cf6, #7c3aed);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.method-content {
  flex-grow: 1;
}

.method-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.75rem;
}

.method-desc {
  margin-top: 0.75rem;
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

/* PATH DEMO */
.path-demo {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.path-header {
  display: flex;
  align-items: center;
  gap: 12px;
  font-size: 1.25rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1.5rem;
}

.path-list {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.path-item {
  display: flex;
  gap: 1rem;
  align-items: center;
  background: rgba(0, 0, 0, 0.3);
  padding: 1rem;
  border-radius: 8px;
}

.path-num {
  min-width: 2rem;
  height: 2rem;
  background: #3b82f6;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.path-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

/* DEBUG TECHNIQUES */
.debug-techniques {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.debug-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.debug-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(59, 130, 246, 0.1);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
}

.debug-content {
  padding: 1.5rem;
}

.debug-note {
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  font-size: 0.85rem;
  color: #94a3b8;
}

/* ERROR EXPANSION */
:deep(.error-header) {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid rgba(239, 68, 68, 0.3);
  border-radius: 8px;
  color: #fca5a5;
}

.error-content {
  background: rgba(15, 23, 42, 0.6);
  padding: 1.5rem;
  color: #cbd5e1;
}

/* VIDEO */
.video-container {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 12px;
  background: #000;
}

.video-wrapper iframe {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}

.video-caption {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  color: #94a3b8;
  font-size: 0.85rem;
}

/* SUMMARY */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.summary-item code {
  font-family: 'Fira Code', monospace;
  color: #22c55e;
  font-size: 1rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 768px) {
  .chmod-demo {
    grid-template-columns: 1fr;
  }

  .chmod-action {
    order: 2;
  }

  .line-tooltip {
    display: none;
  }
}
</style>
