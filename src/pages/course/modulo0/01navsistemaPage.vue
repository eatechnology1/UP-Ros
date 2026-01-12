<template>
  <LessonContainer>
    <!-- ============================================ -->
    <!-- SECCIÓN 1: CONTEXTO E IMPORTANCIA -->
    <!-- ============================================ -->
    <TextBlock>
      Como ingeniero en robótica, el 90% de tu tiempo estarás conectado a robots o servidores que
      <strong>no tienen monitor ni mouse</strong>. Solo tendrás una pantalla negra y tu teclado.
      <br /><br />
      La terminal no es una reliquia del pasado. Es el <strong>lenguaje nativo</strong> de Linux, y
      Linux es el sistema operativo del 96% de los servidores del mundo y del 100% de los robots
      profesionales. Dominarla es la diferencia entre un usuario y un desarrollador.
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué no usar interfaz gráfica?">
      Las interfaces gráficas (GUI) consumen recursos (RAM, CPU) que en un robot son críticos.
      Además, cuando trabajas remotamente vía SSH (conexión a distancia), solo tienes acceso a la
      terminal.
      <br /><br />
      <strong>Regla de oro:</strong> Si puedes hacerlo en la terminal, hazlo en la terminal.
    </AlertBlock>

    <!-- ============================================ -->
    <!-- SECCIÓN 2: ANATOMÍA DEL PROMPT -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>Anatomía del Prompt (La Línea de Comandos)</SectionTitle>
      <TextBlock>
        Antes de ejecutar comandos, debes entender qué significa cada símbolo que ves en la
        terminal.
      </TextBlock>

      <div class="prompt-anatomy q-mt-md">
        <div class="prompt-display">
          <span class="prompt-user">alexander</span>
          <span class="prompt-separator">@</span>
          <span class="prompt-host">robot</span>
          <span class="prompt-separator">:</span>
          <span class="prompt-path">~/ros2_ws</span>
          <span class="prompt-symbol">$</span>
          <span class="prompt-cursor">█</span>
        </div>

        <div class="prompt-legend q-mt-md">
          <div class="legend-item">
            <div class="legend-label text-cyan-4">alexander</div>
            <div class="legend-desc">Nombre de usuario actual</div>
          </div>
          <div class="legend-item">
            <div class="legend-label text-grey-5">@</div>
            <div class="legend-desc">Separador (at)</div>
          </div>
          <div class="legend-item">
            <div class="legend-label text-green-4">robot</div>
            <div class="legend-desc">Nombre del equipo (hostname)</div>
          </div>
          <div class="legend-item">
            <div class="legend-label text-blue-4">~/ros2_ws</div>
            <div class="legend-desc">Directorio actual (~ = /home/alexander)</div>
          </div>
          <div class="legend-item">
            <div class="legend-label text-yellow-4">$</div>
            <div class="legend-desc">Usuario normal (# = root/superusuario)</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 3: FILESYSTEM HIERARCHY -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>El Árbol de Directorios de Linux</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            A diferencia de Windows (C:\, D:\), Linux tiene una sola raíz: <code>/</code>
            <br /><br />
            Todo en el sistema es un archivo o un directorio. Incluso los dispositivos de hardware
            (como tu cámara USB) aparecen como archivos en <code>/dev</code>.
          </TextBlock>

          <AlertBlock type="warning" title="Directorios Críticos para ROS 2">
            <ul class="q-pl-md text-grey-3">
              <li><code>/home</code> — Tu espacio personal. Aquí vive tu workspace de ROS.</li>
              <li><code>/opt/ros</code> — Instalación oficial de ROS 2 (Humble, Foxy, etc.)</li>
              <li><code>/usr/bin</code> — Binarios ejecutables del sistema (python3, gcc)</li>
              <li><code>/tmp</code> — Archivos temporales (se borran al reiniciar)</li>
            </ul>
          </AlertBlock>
        </template>

        <template #right>
          <div class="filesystem-tree">
            <div class="tree-node root">
              <q-icon name="folder" color="yellow-6" size="sm" />
              <span class="node-name">/ (raíz)</span>
            </div>
            <div class="tree-children">
              <div class="tree-branch">
                <div class="tree-node">
                  <q-icon name="folder" color="blue-4" size="xs" />
                  <span class="node-name">/home</span>
                </div>
                <div class="tree-children">
                  <div class="tree-node">
                    <q-icon name="folder" color="cyan-4" size="xs" />
                    <span class="node-name">/alexander</span>
                  </div>
                </div>
              </div>
              <div class="tree-branch">
                <div class="tree-node">
                  <q-icon name="folder" color="green-4" size="xs" />
                  <span class="node-name">/opt</span>
                </div>
                <div class="tree-children">
                  <div class="tree-node">
                    <q-icon name="folder" color="purple-4" size="xs" />
                    <span class="node-name">/ros</span>
                  </div>
                </div>
              </div>
              <div class="tree-branch">
                <div class="tree-node">
                  <q-icon name="folder" color="orange-4" size="xs" />
                  <span class="node-name">/usr</span>
                </div>
              </div>
              <div class="tree-branch">
                <div class="tree-node">
                  <q-icon name="folder" color="red-4" size="xs" />
                  <span class="node-name">/tmp</span>
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 4: COMANDO PWD -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>1. Ubicación Actual: pwd (Print Working Directory)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El comando <code>pwd</code> es tu GPS. Te devuelve la ruta
            <strong>absoluta</strong> desde la raíz (<code>/</code>) hasta donde estás parado.
            <br /><br />
            <strong>¿Cuándo usarlo?</strong> Siempre que te sientas perdido después de varios
            <code>cd</code>, o cuando necesites copiar la ruta exacta para un script.
          </TextBlock>

          <CodeBlock
            title="Ejemplo de uso"
            lang="bash"
            content="alexander@robot:~$ pwd
/home/alexander

alexander@robot:~$ cd /opt/ros/humble
alexander@robot:/opt/ros/humble$ pwd
/opt/ros/humble"
            :copyable="true"
          />
        </template>

        <template #right>
          <div class="concept-card">
            <div class="concept-header">
              <q-icon name="location_on" color="red-5" size="lg" />
              <div class="concept-title">Rutas Absolutas vs Relativas</div>
            </div>
            <div class="concept-body">
              <div class="concept-item">
                <div class="concept-label text-green-4">Absoluta</div>
                <div class="concept-desc">
                  Empieza desde la raíz (<code>/</code>). Funciona desde cualquier lugar.
                  <br />
                  Ejemplo: <code>/home/alexander/ros2_ws/src</code>
                </div>
              </div>
              <div class="concept-divider"></div>
              <div class="concept-item">
                <div class="concept-label text-blue-4">Relativa</div>
                <div class="concept-desc">
                  Empieza desde donde estás. Más corta pero depende del contexto.
                  <br />
                  Ejemplo: <code>../install/setup.bash</code>
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 5: COMANDO LS (EXPANDIDO) -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>2. Exploración: ls (List Directory Contents)</SectionTitle>
      <TextBlock>
        <code>ls</code> lista el contenido del directorio actual. Pero en ingeniería, los detalles
        importan. Aquí están las variantes que usarás a diario:
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="command-card">
            <div class="command-header">
              <q-icon name="list" color="cyan-4" size="md" />
              <code class="command-name">ls -l</code>
            </div>
            <div class="command-desc">
              Formato de lista <strong>detallada</strong>. Muestra permisos (rwx), propietario,
              tamaño y fecha de modificación.
            </div>
            <div class="command-example">
              <CodeBlock
                :hide-header="true"
                lang="bash"
                content="$ ls -l
-rw-r--r-- 1 alexander users  1024 Jan 10 14:30 robot.py
drwxr-xr-x 2 alexander users  4096 Jan 10 14:25 src/"
              />
            </div>
            <div class="command-use-case">
              <strong>Caso de uso:</strong> Verificar si un script tiene permisos de ejecución antes
              de lanzarlo.
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="command-card">
            <div class="command-header">
              <q-icon name="visibility" color="purple-4" size="md" />
              <code class="command-name">ls -a</code>
            </div>
            <div class="command-desc">
              Muestra archivos <strong>ocultos</strong> (los que empiezan con <code>.</code>). Aquí
              viven configuraciones críticas.
            </div>
            <div class="command-example">
              <CodeBlock
                :hide-header="true"
                lang="bash"
                content="$ ls -a
.  ..  .bashrc  .gitignore  robot.py  src/"
              />
            </div>
            <div class="command-use-case">
              <strong>Caso de uso:</strong> Editar <code>.bashrc</code> para configurar variables de
              entorno de ROS 2.
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="command-card">
            <div class="command-header">
              <q-icon name="straighten" color="green-4" size="md" />
              <code class="command-name">ls -lh</code>
            </div>
            <div class="command-desc">
              Tamaños en formato <strong>legible</strong> (KB, MB, GB) en lugar de bytes.
            </div>
            <div class="command-example">
              <CodeBlock
                :hide-header="true"
                lang="bash"
                content="$ ls -lh
-rw-r--r-- 1 alexander users  1.0K Jan 10 14:30 robot.py
-rw-r--r-- 1 alexander users  2.3M Jan 10 14:28 rosbag.db3"
              />
            </div>
            <div class="command-use-case">
              <strong>Caso de uso:</strong> Verificar el tamaño de un rosbag antes de transferirlo.
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="command-card">
            <div class="command-header">
              <q-icon name="filter_list" color="orange-4" size="md" />
              <code class="command-name">ls *.py</code>
            </div>
            <div class="command-desc">
              Usa <strong>wildcards</strong> (<code>*</code>) para filtrar. El asterisco significa
              "cualquier cosa".
            </div>
            <div class="command-example">
              <CodeBlock
                :hide-header="true"
                lang="bash"
                content="$ ls *.py
robot.py  sensor_node.py  controller.py"
              />
            </div>
            <div class="command-use-case">
              <strong>Caso de uso:</strong> Listar solo archivos Python en un directorio con muchos
              tipos de archivos.
            </div>
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="Combinación Pro: ls -lah">
        Combina las tres banderas más útiles:
        <code>-l</code> (detallado) + <code>-a</code> (ocultos) + <code>-h</code> (legible).
        <br />
        Este es el comando que usarás el 80% del tiempo.
      </AlertBlock>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 6: COMANDO CD (EXPANDIDO) -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>3. Movimiento Rápido: cd (Change Directory)</SectionTitle>
      <TextBlock>
        La terminal tiene atajos de "teletransportación" que te ahorrarán horas de escritura. Domina
        estos y tu velocidad se multiplicará.
      </TextBlock>

      <div class="shortcuts-grid q-mt-md">
        <div class="shortcut-card">
          <div class="shortcut-command">cd ..</div>
          <div class="shortcut-desc">Subir un nivel (ir a la carpeta padre)</div>
          <div class="shortcut-example">
            <code>/home/alexander/ros2_ws/src</code> → <code>/home/alexander/ros2_ws</code>
          </div>
        </div>

        <div class="shortcut-card">
          <div class="shortcut-command">cd ~</div>
          <div class="shortcut-desc">Ir directamente a tu carpeta Home</div>
          <div class="shortcut-example">Desde cualquier lugar → <code>/home/alexander</code></div>
        </div>

        <div class="shortcut-card">
          <div class="shortcut-command">cd -</div>
          <div class="shortcut-desc">Volver a la carpeta anterior (como "atrás" del navegador)</div>
          <div class="shortcut-example">
            Alterna entre <code>/opt/ros/humble</code> ↔ <code>~/ros2_ws</code>
          </div>
        </div>

        <div class="shortcut-card">
          <div class="shortcut-command">cd</div>
          <div class="shortcut-desc">Sin argumentos, también te lleva a Home</div>
          <div class="shortcut-example">Equivalente a <code>cd ~</code></div>
        </div>

        <div class="shortcut-card">
          <div class="shortcut-command">cd /</div>
          <div class="shortcut-desc">Ir a la raíz del sistema</div>
          <div class="shortcut-example">El punto de partida de todo el filesystem</div>
        </div>

        <div class="shortcut-card">
          <div class="shortcut-command">cd ../..</div>
          <div class="shortcut-desc">Subir dos niveles de una vez</div>
          <div class="shortcut-example">
            <code>/home/alexander/ros2_ws/src/pkg</code> → <code>/home/alexander/ros2_ws</code>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SplitBlock>
          <template #left>
            <AlertBlock type="success" title="Tip de Productividad: Autocompletado">
              Usa la tecla <strong>TAB</strong> para autocompletar nombres. <br /><br />
              Escribe <code>cd rob</code> + TAB → se convierte en <code>cd robot_ws/</code>
              <br />
              Si hay múltiples coincidencias, presiona TAB dos veces para ver las opciones.
            </AlertBlock>
          </template>

          <template #right>
            <AlertBlock type="info" title="Historial de Comandos">
              Usa las flechas <strong>↑</strong> y <strong>↓</strong> para navegar por comandos
              anteriores. <br /><br />
              Presiona <strong>Ctrl + R</strong> y escribe parte de un comando para buscarlo en el
              historial.
            </AlertBlock>
          </template>
        </SplitBlock>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 7: ERRORES COMUNES -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>Errores Comunes y Soluciones</SectionTitle>

      <div class="error-accordion">
        <q-expansion-item
          icon="error"
          label="bash: cd: directorio: No such file or directory"
          header-class="error-header"
        >
          <div class="error-content">
            <div class="error-cause">
              <strong>Causa:</strong> Escribiste mal el nombre o el directorio no existe.
            </div>
            <div class="error-solution">
              <strong>Solución:</strong>
              <ol>
                <li>Usa <code>ls</code> para ver qué directorios existen</li>
                <li>Usa TAB para autocompletar y evitar errores de tipeo</li>
                <li>Verifica mayúsculas/minúsculas (Linux es case-sensitive)</li>
              </ol>
            </div>
          </div>
        </q-expansion-item>

        <q-expansion-item
          icon="error"
          label="bash: ls: cannot access 'archivo.txt': Permission denied"
          header-class="error-header"
        >
          <div class="error-content">
            <div class="error-cause">
              <strong>Causa:</strong> No tienes permisos para leer ese archivo o directorio.
            </div>
            <div class="error-solution">
              <strong>Solución:</strong>
              <ul>
                <li>Verifica permisos con <code>ls -l</code></li>
                <li>Si es tu archivo, usa <code>chmod +r archivo.txt</code></li>
                <li>Si es del sistema, usa <code>sudo ls archivo.txt</code> (con precaución)</li>
              </ul>
            </div>
          </div>
        </q-expansion-item>

        <q-expansion-item
          icon="error"
          label="Me perdí en el sistema, ¿cómo vuelvo a mi carpeta?"
          header-class="error-header"
        >
          <div class="error-content">
            <div class="error-cause">
              <strong>Situación:</strong> Hiciste varios <code>cd</code> y no sabes dónde estás.
            </div>
            <div class="error-solution">
              <strong>Solución rápida:</strong>
              <ol>
                <li>Ejecuta <code>pwd</code> para ver tu ubicación actual</li>
                <li>Ejecuta <code>cd ~</code> para volver a tu Home</li>
                <li>Usa <code>cd -</code> para volver al directorio anterior</li>
              </ol>
            </div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 8: RETO PRÁCTICO -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Ingeniería</SectionTitle>
      <TextBlock>
        Predice la salida final antes de ejecutar este bloque. ¿En qué carpeta terminarás? ¿Qué
        archivos se habrán creado?
      </TextBlock>

      <CodeBlock
        title="Simulación de Navegación"
        lang="bash"
        content="cd ~
mkdir -p proyecto_xyz/logs
cd proyecto_xyz/logs
touch error.log debug.log
ls -lh
cd ../..
pwd
ls"
        :copyable="true"
      />

      <AlertBlock type="info" title="Respuesta (no hagas trampa)">
        <strong>Directorio final:</strong> <code>/home/alexander</code> (tu Home)
        <br />
        <strong>Archivos creados:</strong> <code>error.log</code> y <code>debug.log</code> dentro de
        <code>~/proyecto_xyz/logs/</code>
        <br />
        <strong>Explicación:</strong> <code>cd ../..</code> sube dos niveles desde
        <code>logs/</code> → <code>proyecto_xyz/</code> → <code>~</code>
      </AlertBlock>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 9: VIDEO COMPLEMENTARIO -->
    <!-- ============================================ -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <TextBlock>
        Refuerza lo aprendido con este video tutorial que demuestra estos comandos en acción:
      </TextBlock>

      <div class="video-container q-mt-md">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Navegación en Terminal Linux"
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
          <q-icon name="info" color="blue-4" size="sm" class="q-mr-sm" />
          Video en Progreso
        </div>
      </div>
    </div>

    <!-- ============================================ -->
    <!-- SECCIÓN 10: RESUMEN Y PRÓXIMOS PASOS -->
    <!-- ============================================ -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen de Comandos Esenciales</SectionTitle>

      <div class="summary-table">
        <div class="summary-row summary-header">
          <div class="summary-cell">Comando</div>
          <div class="summary-cell">Descripción</div>
          <div class="summary-cell">Ejemplo</div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>pwd</code></div>
          <div class="summary-cell">Muestra tu ubicación actual</div>
          <div class="summary-cell"><code>pwd</code></div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>ls -lah</code></div>
          <div class="summary-cell">Lista archivos (detallado + ocultos + legible)</div>
          <div class="summary-cell"><code>ls -lah</code></div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>cd ~</code></div>
          <div class="summary-cell">Ir a tu carpeta Home</div>
          <div class="summary-cell"><code>cd ~</code></div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>cd ..</code></div>
          <div class="summary-cell">Subir un nivel</div>
          <div class="summary-cell"><code>cd ..</code></div>
        </div>
        <div class="summary-row">
          <div class="summary-cell"><code>cd -</code></div>
          <div class="summary-cell">Volver al directorio anterior</div>
          <div class="summary-cell"><code>cd -</code></div>
        </div>
      </div>

      <AlertBlock type="success" title="Próximo Paso" class="q-mt-lg">
        Ahora que dominas la navegación, el siguiente módulo te enseñará a
        <strong>crear, copiar, mover y eliminar</strong> archivos y directorios. ¡Prepárate para
        construir tu primer workspace de ROS 2!
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
import SplitBlock from 'components/content/SplitBlock.vue';
</script>

<style scoped>
/* ============================================ */
/* LAYOUT GENERAL */
/* ============================================ */
.section-group {
  margin-bottom: 3.5rem;
}

/* ============================================ */
/* PROMPT ANATOMY */
/* ============================================ */
.prompt-anatomy {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.prompt-display {
  background: #1e1e1e;
  border: 1px solid #333;
  border-radius: 8px;
  padding: 1.5rem;
  font-family: 'Fira Code', monospace;
  font-size: 1.1rem;
  display: flex;
  align-items: center;
  gap: 4px;
}

.prompt-user {
  color: #06b6d4;
  font-weight: 600;
}

.prompt-separator {
  color: #64748b;
}

.prompt-host {
  color: #22c55e;
  font-weight: 600;
}

.prompt-path {
  color: #3b82f6;
}

.prompt-symbol {
  color: #facc15;
  margin-left: 8px;
  margin-right: 8px;
}

.prompt-cursor {
  color: #f1f5f9;
  animation: blink 1s infinite;
}

@keyframes blink {
  0%,
  50% {
    opacity: 1;
  }
  51%,
  100% {
    opacity: 0;
  }
}

.prompt-legend {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.legend-item {
  background: rgba(255, 255, 255, 0.03);
  border-radius: 8px;
  padding: 0.75rem;
}

.legend-label {
  font-family: 'Fira Code', monospace;
  font-weight: 700;
  margin-bottom: 0.25rem;
}

.legend-desc {
  font-size: 0.85rem;
  color: #cbd5e1;
}

/* ============================================ */
/* FILESYSTEM TREE */
/* ============================================ */
.filesystem-tree {
  background: #0f172a;
  border: 1px solid #1e293b;
  border-radius: 12px;
  padding: 1.5rem;
  font-family: 'Fira Code', monospace;
}

.tree-node {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 6px 12px;
  margin: 4px 0;
  border-radius: 6px;
  transition: background 0.2s;
}

.tree-node:hover {
  background: rgba(59, 130, 246, 0.1);
}

.tree-node.root {
  background: rgba(234, 179, 8, 0.1);
  border-left: 3px solid #eab308;
}

.node-name {
  color: #e2e8f0;
  font-size: 0.95rem;
}

.tree-children {
  margin-left: 24px;
  border-left: 2px dashed #334155;
  padding-left: 12px;
}

.tree-branch {
  margin: 8px 0;
}

/* ============================================ */
/* CONCEPT CARD */
/* ============================================ */
.concept-card {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
  height: 100%;
}

.concept-header {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 1.5rem;
}

.concept-title {
  font-size: 1.25rem;
  font-weight: 700;
  color: #f1f5f9;
}

.concept-body {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.concept-item {
  padding: 1rem;
  background: rgba(255, 255, 255, 0.03);
  border-radius: 8px;
}

.concept-label {
  font-weight: 700;
  font-size: 1rem;
  margin-bottom: 0.5rem;
}

.concept-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

.concept-divider {
  height: 1px;
  background: linear-gradient(90deg, transparent, #334155, transparent);
}

/* ============================================ */
/* COMMAND CARDS */
/* ============================================ */
.command-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.25rem;
  height: 100%;
  display: flex;
  flex-direction: column;
  gap: 1rem;
  transition:
    transform 0.2s,
    border-color 0.2s;
}

.command-card:hover {
  transform: translateY(-4px);
  border-color: rgba(59, 130, 246, 0.5);
}

.command-header {
  display: flex;
  align-items: center;
  gap: 12px;
}

.command-name {
  font-family: 'Fira Code', monospace;
  font-size: 1.1rem;
  color: #fbbf24;
  font-weight: 700;
}

.command-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
  line-height: 1.5;
}

.command-example {
  flex-grow: 1;
}

.command-use-case {
  background: rgba(59, 130, 246, 0.1);
  border-left: 3px solid #3b82f6;
  padding: 0.75rem;
  border-radius: 4px;
  font-size: 0.85rem;
  color: #94a3b8;
}

/* ============================================ */
/* SHORTCUTS GRID */
/* ============================================ */
.shortcuts-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1rem;
}

.shortcut-card {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.8), rgba(30, 41, 59, 0.8));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.25rem;
  transition:
    transform 0.2s,
    box-shadow 0.2s;
}

.shortcut-card:hover {
  transform: translateY(-2px);
  box-shadow: 0 8px 24px rgba(59, 130, 246, 0.2);
}

.shortcut-command {
  font-family: 'Fira Code', monospace;
  font-size: 1.25rem;
  font-weight: 700;
  color: #22c55e;
  margin-bottom: 0.5rem;
}

.shortcut-desc {
  color: #e2e8f0;
  font-size: 0.95rem;
  margin-bottom: 0.75rem;
}

.shortcut-example {
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  padding: 0.5rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
  color: #94a3b8;
}

/* ============================================ */
/* ERROR ACCORDION */
/* ============================================ */
.error-accordion {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

:deep(.error-header) {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid rgba(239, 68, 68, 0.3);
  border-radius: 8px;
  color: #fca5a5;
}

.error-content {
  background: rgba(15, 23, 42, 0.6);
  padding: 1.5rem;
  border-radius: 0 0 8px 8px;
}

.error-cause {
  margin-bottom: 1rem;
  color: #fca5a5;
}

.error-solution {
  color: #cbd5e1;
}

.error-solution ol,
.error-solution ul {
  margin-top: 0.5rem;
  padding-left: 1.5rem;
}

.error-solution li {
  margin-bottom: 0.5rem;
}

/* ============================================ */
/* VIDEO CONTAINER */
/* ============================================ */
.video-container {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%; /* 16:9 aspect ratio */
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
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  color: #94a3b8;
  font-size: 0.85rem;
}

/* ============================================ */
/* SUMMARY TABLE */
/* ============================================ */
.summary-table {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.summary-row {
  display: grid;
  grid-template-columns: 1fr 2fr 1.5fr;
  gap: 1rem;
  padding: 1rem 1.5rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
}

.summary-row:last-child {
  border-bottom: none;
}

.summary-header {
  background: rgba(59, 130, 246, 0.1);
  font-weight: 700;
  color: #60a5fa;
}

.summary-cell {
  display: flex;
  align-items: center;
  color: #e2e8f0;
  font-size: 0.95rem;
}

.summary-cell code {
  background: rgba(0, 0, 0, 0.3);
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  font-family: 'Fira Code', monospace;
  color: #22c55e;
}

/* ============================================ */
/* RESPONSIVE */
/* ============================================ */
@media (max-width: 768px) {
  .prompt-display {
    font-size: 0.85rem;
    flex-wrap: wrap;
  }

  .summary-row {
    grid-template-columns: 1fr;
    gap: 0.5rem;
  }

  .shortcuts-grid {
    grid-template-columns: 1fr;
  }
}
</style>
