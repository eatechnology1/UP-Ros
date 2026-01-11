<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      El código no se escribe en el aire. Necesitas un editor de texto. Pero no cualquier editor:
      uno que entienda Python, C++, YAML, y que funcione tanto en tu laptop como en un robot sin
      pantalla conectado por SSH a 10,000 km de distancia.
      <br /><br />
      Esta lección te enseña <strong>tres herramientas</strong> que dominarás en diferentes
      contextos: <strong>nano</strong> (emergencias), <strong>VS Code</strong> (desarrollo diario),
      y <strong>vim</strong> (opcional, para valientes).
    </TextBlock>

    <!-- LA TRINIDAD -->
    <div class="section-group">
      <SectionTitle>1. La Trinidad de Editores</SectionTitle>

      <div class="editors-grid">
        <div class="editor-card nano">
          <div class="editor-icon">🛡️</div>
          <div class="editor-name">GNU nano</div>
          <div class="editor-tagline">"El salvavidas"</div>
          <div class="editor-stats">
            <div class="stat-item">
              <div class="stat-label">Curva de aprendizaje</div>
              <div class="stat-value">30 segundos</div>
            </div>
            <div class="stat-item">
              <div class="stat-label">Uso principal</div>
              <div class="stat-value">Ediciones rápidas por SSH</div>
            </div>
          </div>
          <div class="editor-when">
            <strong>Cuándo usarlo:</strong> Editar <code>.bashrc</code>, configurar redes, arreglar
            archivos en robots remotos
          </div>
        </div>

        <div class="editor-card vscode">
          <div class="editor-icon">💻</div>
          <div class="editor-name">VS Code</div>
          <div class="editor-tagline">"El taller completo"</div>
          <div class="editor-stats">
            <div class="stat-item">
              <div class="stat-label">Curva de aprendizaje</div>
              <div class="stat-value">1 semana</div>
            </div>
            <div class="stat-item">
              <div class="stat-label">Uso principal</div>
              <div class="stat-value">Desarrollo de paquetes ROS 2</div>
            </div>
          </div>
          <div class="editor-when">
            <strong>Cuándo usarlo:</strong> Escribir nodos Python/C++, debugging, Git, desarrollo
            local (95% del tiempo)
          </div>
        </div>

        <div class="editor-card vim">
          <div class="editor-icon">💀</div>
          <div class="editor-name">Vim</div>
          <div class="editor-tagline">"El legendario"</div>
          <div class="editor-stats">
            <div class="stat-item">
              <div class="stat-label">Curva de aprendizaje</div>
              <div class="stat-value">6 meses - ∞</div>
            </div>
            <div class="stat-item">
              <div class="stat-label">Uso principal</div>
              <div class="stat-value">Edición ultra-rápida (expertos)</div>
            </div>
          </div>
          <div class="editor-when">
            <strong>Cuándo usarlo:</strong> Cuando quieras impresionar (o torturarte). Opcional para
            este curso.
          </div>
        </div>
      </div>
    </div>

    <!-- NANO PROFUNDO -->
    <div class="section-group">
      <SectionTitle>2. Dominando nano: Tu Navaja Suiza</SectionTitle>
      <TextBlock>
        <code>nano</code> es el editor que <strong>siempre funciona</strong>. No importa si estás en
        una Raspberry Pi sin GUI, en un contenedor Docker, o conectado por SSH a un satélite. Si hay
        terminal, hay nano.
      </TextBlock>

      <div class="nano-demo q-mt-md">
        <div class="nano-header">
          <div class="nano-title">GNU nano 6.2</div>
          <div class="nano-file">Archivo: robot_config.yaml</div>
        </div>
        <div class="nano-content">
          <div class="nano-line">
            <span class="line-num">1</span>
            <span class="code-text">ros_domain_id: 30</span>
          </div>
          <div class="nano-line">
            <span class="line-num">2</span>
            <span class="code-text">node_name: "control_node"</span>
          </div>
          <div class="nano-line">
            <span class="line-num">3</span>
            <span class="code-text">max_speed: 1.5<span class="cursor">█</span></span>
          </div>
        </div>
        <div class="nano-shortcuts">
          <div class="shortcut"><span class="key">^G</span> Ayuda</div>
          <div class="shortcut"><span class="key">^O</span> Guardar</div>
          <div class="shortcut"><span class="key">^X</span> Salir</div>
          <div class="shortcut"><span class="key">^K</span> Cortar</div>
          <div class="shortcut"><span class="key">^U</span> Pegar</div>
          <div class="shortcut"><span class="key">^W</span> Buscar</div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Flujo de Trabajo con nano</SectionTitle>
        <div class="workflow-steps">
          <div class="workflow-step">
            <div class="step-number">1</div>
            <div class="step-content">
              <div class="step-title">Abrir archivo</div>
              <CodeBlock lang="bash" content="nano config.yaml" :copyable="true" />
            </div>
          </div>
          <div class="workflow-step">
            <div class="step-number">2</div>
            <div class="step-content">
              <div class="step-title">Editar</div>
              <TextBlock>Escribe normalmente. Las flechas funcionan como esperas.</TextBlock>
            </div>
          </div>
          <div class="workflow-step">
            <div class="step-number">3</div>
            <div class="step-content">
              <div class="step-title">Guardar</div>
              <TextBlock>
                <code>Ctrl + O</code> → Enter (confirmar nombre) → Archivo guardado
              </TextBlock>
            </div>
          </div>
          <div class="workflow-step">
            <div class="step-number">4</div>
            <div class="step-content">
              <div class="step-title">Salir</div>
              <TextBlock><code>Ctrl + X</code> (si no guardaste, te preguntará)</TextBlock>
            </div>
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="Tip Pro: Crear y Editar en un Comando" class="q-mt-md">
        Si el archivo no existe, nano lo crea automáticamente:
        <br />
        <code>nano nuevo_nodo.py</code> → Escribe código → Ctrl+O → Ctrl+X
      </AlertBlock>
    </div>

    <!-- VS CODE PROFUNDO -->
    <div class="section-group">
      <SectionTitle>3. VS Code: Tu IDE de Batalla para ROS 2</SectionTitle>
      <TextBlock>
        Visual Studio Code es <strong>el estándar de facto</strong> para desarrollo ROS 2. Es
        gratis, potente, y tiene extensiones para todo. Aquí está tu setup profesional:
      </TextBlock>

      <div class="vscode-setup q-mt-md">
        <div class="setup-section">
          <div class="setup-title">
            <q-icon name="extension" color="blue-4" size="md" />
            <span>Extensiones Esenciales</span>
          </div>
          <div class="extensions-grid">
            <div class="extension-card">
              <div class="ext-name">Python</div>
              <div class="ext-author">Microsoft</div>
              <div class="ext-desc">IntelliSense, debugging, linting para nodos Python</div>
            </div>
            <div class="extension-card">
              <div class="ext-name">C/C++</div>
              <div class="ext-author">Microsoft</div>
              <div class="ext-desc">Autocompletado para nodos C++, integración con CMake</div>
            </div>
            <div class="extension-card">
              <div class="ext-name">ROS</div>
              <div class="ext-author">Microsoft</div>
              <div class="ext-desc">Syntax highlighting para .msg, .srv, .action, launch files</div>
            </div>
            <div class="extension-card">
              <div class="ext-name">Remote - SSH</div>
              <div class="ext-author">Microsoft</div>
              <div class="ext-desc">Editar código en robots remotos como si fueran locales</div>
            </div>
            <div class="extension-card">
              <div class="ext-name">YAML</div>
              <div class="ext-author">Red Hat</div>
              <div class="ext-desc">Validación de sintaxis para archivos de configuración</div>
            </div>
            <div class="extension-card">
              <div class="ext-name">GitLens</div>
              <div class="ext-author">GitKraken</div>
              <div class="ext-desc">Superpoderes para Git (blame, history, comparaciones)</div>
            </div>
          </div>
        </div>

        <div class="setup-section q-mt-lg">
          <div class="setup-title">
            <q-icon name="settings" color="orange-4" size="md" />
            <span>Configuración Recomendada</span>
          </div>
          <CodeBlock
            title="settings.json (Ctrl+Shift+P → Preferences: Open Settings JSON)"
            lang="json"
            content='{
  "python.defaultInterpreterPath": "/usr/bin/python3",
  "python.autoComplete.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages"
  ],
  "files.associations": {
    "*.launch": "xml",
    "*.world": "xml",
    "*.urdf": "xml",
    "*.xacro": "xml"
  },
  "editor.formatOnSave": true,
  "editor.rulers": [100],
  "terminal.integrated.shell.linux": "/bin/bash"
}'
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- REMOTE SSH -->
    <div class="section-group">
      <SectionTitle>4. VS Code Remote SSH: El Puente Mágico</SectionTitle>
      <TextBlock>
        Imagina editar código en un robot físico (Raspberry Pi, Jetson Nano) con la comodidad de VS
        Code en tu laptop. <strong>Remote SSH</strong> hace exactamente eso.
      </TextBlock>

      <div class="remote-diagram q-mt-md">
        <div class="remote-local">
          <q-icon name="laptop_mac" size="4rem" color="blue-4" />
          <div class="remote-label">Tu Laptop</div>
          <div class="remote-desc">VS Code con interfaz gráfica</div>
        </div>
        <div class="remote-connection">
          <q-icon name="swap_horiz" size="2rem" color="green-4" />
          <div class="connection-label">SSH (puerto 22)</div>
        </div>
        <div class="remote-robot">
          <q-icon name="memory" size="4rem" color="green-4" />
          <div class="remote-label">Robot (Raspberry Pi)</div>
          <div class="remote-desc">Código ejecutándose aquí</div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Cómo Conectarte</SectionTitle>
        <div class="steps-compact">
          <div class="step-compact">
            <strong>1.</strong> Instala extensión "Remote - SSH" en VS Code
          </div>
          <div class="step-compact">
            <strong>2.</strong> Presiona <code>F1</code> → "Remote-SSH: Connect to Host"
          </div>
          <div class="step-compact">
            <strong>3.</strong> Escribe: <code>usuario@192.168.1.100</code> (IP de tu robot)
          </div>
          <div class="step-compact">
            <strong>4.</strong> Ingresa contraseña → ¡Listo! Ahora editas archivos remotos
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="Ventaja Clave" class="q-mt-md">
        Con Remote SSH, puedes usar <strong>toda la potencia de VS Code</strong> (debugging,
        IntelliSense, Git) en código que vive en el robot. Es como tener dos mundos en uno.
      </AlertBlock>
    </div>

    <!-- VIM (OPCIONAL) -->
    <div class="section-group">
      <SectionTitle>5. Vim: El Camino del Guerrero (Opcional)</SectionTitle>
      <TextBlock>
        <code>vim</code> es legendario por su eficiencia... y por su curva de aprendizaje vertical.
        <strong>No es obligatorio para este curso</strong>, pero si quieres aprenderlo, aquí está lo
        mínimo para sobrevivir:
      </TextBlock>

      <div class="vim-survival q-mt-md">
        <div class="survival-title">🆘 Kit de Supervivencia Vim</div>
        <div class="survival-grid">
          <div class="survival-item">
            <div class="survival-cmd">vim archivo.txt</div>
            <div class="survival-desc">Abrir archivo</div>
          </div>
          <div class="survival-item">
            <div class="survival-cmd">i</div>
            <div class="survival-desc">Entrar a modo INSERT (ahora puedes escribir)</div>
          </div>
          <div class="survival-item">
            <div class="survival-cmd">ESC</div>
            <div class="survival-desc">Salir del modo INSERT (volver a modo COMMAND)</div>
          </div>
          <div class="survival-item">
            <div class="survival-cmd">:w</div>
            <div class="survival-desc">Guardar (write)</div>
          </div>
          <div class="survival-item">
            <div class="survival-cmd">:q</div>
            <div class="survival-desc">Salir (quit)</div>
          </div>
          <div class="survival-item danger">
            <div class="survival-cmd">:q!</div>
            <div class="survival-desc">Salir SIN guardar (forzar)</div>
          </div>
        </div>
      </div>

      <AlertBlock type="warning" title="La Leyenda Urbana" class="q-mt-md">
        Se dice que hay programadores atrapados en Vim desde 1995 porque no saben cómo salir. La
        respuesta es <code>:q!</code> (dos puntos, q, signo de exclamación, Enter).
      </AlertBlock>
    </div>

    <!-- CASOS DE USO -->
    <div class="section-group">
      <SectionTitle>6. Casos de Uso Reales en ROS 2</SectionTitle>

      <div class="use-case-grid">
        <div class="use-case-item">
          <div class="use-case-scenario">
            <q-icon name="bug_report" color="red-4" size="md" />
            <span>Tu robot falla en campo</span>
          </div>
          <div class="use-case-solution">
            <strong>Solución:</strong> SSH al robot → <code>nano /var/log/ros.log</code> → Revisar
            errores → Editar config → Reiniciar nodo
          </div>
          <div class="use-case-tool">Herramienta: <strong>nano</strong></div>
        </div>

        <div class="use-case-item">
          <div class="use-case-scenario">
            <q-icon name="code" color="blue-4" size="md" />
            <span>Desarrollar un nodo nuevo de navegación</span>
          </div>
          <div class="use-case-solution">
            <strong>Solución:</strong> Abrir VS Code → Crear paquete → Escribir Python/C++ con
            IntelliSense → Debugger integrado → Git commit
          </div>
          <div class="use-case-tool">Herramienta: <strong>VS Code</strong></div>
        </div>

        <div class="use-case-item">
          <div class="use-case-scenario">
            <q-icon name="network_check" color="orange-4" size="md" />
            <span>Configurar red en robot sin monitor</span>
          </div>
          <div class="use-case-solution">
            <strong>Solución:</strong> Conectar teclado directo al robot →
            <code>sudo nano /etc/netplan/01-netcfg.yaml</code> → Configurar WiFi → Guardar
          </div>
          <div class="use-case-tool">Herramienta: <strong>nano</strong></div>
        </div>

        <div class="use-case-item">
          <div class="use-case-scenario">
            <q-icon name="cloud" color="green-4" size="md" />
            <span>Editar código en robot remoto cómodamente</span>
          </div>
          <div class="use-case-solution">
            <strong>Solución:</strong> VS Code → Remote SSH → Conectar a robot → Editar archivos
            como si fueran locales → Ejecutar desde terminal integrada
          </div>
          <div class="use-case-tool">Herramienta: <strong>VS Code Remote SSH</strong></div>
        </div>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/dQw4w9WgXcQ"
            title="Editores de Texto en Linux"
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
          Reemplaza dQw4w9WgXcQ con tu video de YouTube
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen: Cuándo Usar Cada Editor</SectionTitle>
      <div class="summary-table">
        <div class="summary-row header">
          <div>Editor</div>
          <div>Cuándo Usarlo</div>
          <div>Comando</div>
        </div>
        <div class="summary-row">
          <div><strong>nano</strong></div>
          <div>Ediciones rápidas por SSH, emergencias, configs del sistema</div>
          <div><code>nano archivo.txt</code></div>
        </div>
        <div class="summary-row">
          <div><strong>VS Code</strong></div>
          <div>Desarrollo diario, debugging, proyectos grandes</div>
          <div><code>code .</code></div>
        </div>
        <div class="summary-row">
          <div><strong>VS Code Remote</strong></div>
          <div>Editar código en robots remotos con comodidad</div>
          <div>F1 → Remote-SSH: Connect</div>
        </div>
        <div class="summary-row">
          <div><strong>vim</strong></div>
          <div>Opcional: edición ultra-rápida para expertos</div>
          <div><code>vim archivo.txt</code></div>
        </div>
      </div>

      <AlertBlock type="success" title="Regla de Oro" class="q-mt-lg">
        <strong>95% del tiempo:</strong> VS Code
        <br />
        <strong>5% del tiempo:</strong> nano (cuando VS Code no está disponible)
        <br />
        <strong>0% del tiempo:</strong> vim (a menos que te guste el dolor)
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

.editors-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.editor-card {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  text-align: center;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.editor-card.nano {
  border-top: 4px solid #22c55e;
}
.editor-card.vscode {
  border-top: 4px solid #3b82f6;
}
.editor-card.vim {
  border-top: 4px solid #ef4444;
}

.editor-icon {
  font-size: 3rem;
}

.editor-name {
  font-size: 1.5rem;
  font-weight: 700;
  color: #f1f5f9;
}

.editor-tagline {
  font-style: italic;
  color: #94a3b8;
}

.editor-stats {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  margin: 1rem 0;
}

.stat-item {
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  padding: 0.75rem;
}

.stat-label {
  font-size: 0.75rem;
  color: #94a3b8;
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.stat-value {
  font-size: 1rem;
  font-weight: 700;
  color: #e2e8f0;
  margin-top: 0.25rem;
}

.editor-when {
  text-align: left;
  font-size: 0.9rem;
  color: #cbd5e1;
  background: rgba(59, 130, 246, 0.1);
  padding: 1rem;
  border-radius: 8px;
}

.nano-demo {
  background: #1e1e1e;
  border: 1px solid #444;
  border-radius: 12px;
  overflow: hidden;
  font-family: 'Fira Code', monospace;
}

.nano-header {
  background: #2d2d2d;
  padding: 0.75rem 1.5rem;
  display: flex;
  justify-content: space-between;
  border-bottom: 1px solid #444;
  color: #94a3b8;
  font-size: 0.85rem;
}

.nano-content {
  padding: 1.5rem;
  min-height: 120px;
}

.nano-line {
  display: flex;
  gap: 1rem;
  margin-bottom: 0.5rem;
  color: #22c55e;
}

.line-num {
  color: #64748b;
  min-width: 2ch;
  text-align: right;
}

.cursor {
  background: #22c55e;
  color: #1e1e1e;
  animation: blink 1s step-end infinite;
}

@keyframes blink {
  50% {
    opacity: 0;
  }
}

.nano-shortcuts {
  background: #2d2d2d;
  padding: 1rem 1.5rem;
  border-top: 1px solid #444;
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
  gap: 0.75rem;
  font-size: 0.85rem;
  color: #cbd5e1;
}

.key {
  background: #f1f5f9;
  color: #1e293b;
  padding: 0.125rem 0.5rem;
  border-radius: 4px;
  font-weight: 700;
  margin-right: 0.5rem;
}

.workflow-steps {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.workflow-step {
  display: flex;
  gap: 1.5rem;
  align-items: flex-start;
}

.step-number {
  min-width: 3rem;
  height: 3rem;
  background: linear-gradient(135deg, #3b82f6, #2563eb);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.step-content {
  flex-grow: 1;
}

.step-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}

.vscode-setup {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.setup-title {
  display: flex;
  align-items: center;
  gap: 12px;
  font-size: 1.25rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1.5rem;
}

.extensions-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1rem;
}

.extension-card {
  background: rgba(0, 0, 0, 0.3);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
}

.ext-name {
  font-weight: 700;
  color: #60a5fa;
  margin-bottom: 0.25rem;
}

.ext-author {
  font-size: 0.75rem;
  color: #94a3b8;
  margin-bottom: 0.5rem;
}

.ext-desc {
  font-size: 0.85rem;
  color: #cbd5e1;
}

.remote-diagram {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 2rem;
  padding: 2rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
}

.remote-local,
.remote-robot {
  text-align: center;
}

.remote-label {
  font-weight: 700;
  color: #f1f5f9;
  margin-top: 0.75rem;
}

.remote-desc {
  font-size: 0.85rem;
  color: #94a3b8;
  margin-top: 0.25rem;
}

.remote-connection {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
}

.connection-label {
  font-size: 0.75rem;
  color: #22c55e;
  font-family: 'Fira Code', monospace;
  background: rgba(34, 197, 94, 0.1);
  padding: 0.25rem 0.75rem;
  border-radius: 4px;
}

.steps-compact {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.step-compact {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.vim-survival {
  background: rgba(127, 29, 29, 0.2);
  border: 2px solid rgba(239, 68, 68, 0.5);
  border-radius: 16px;
  padding: 2rem;
}

.survival-title {
  text-align: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: #fca5a5;
  margin-bottom: 1.5rem;
}

.survival-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.survival-item {
  background: rgba(0, 0, 0, 0.3);
  border: 1px solid rgba(239, 68, 68, 0.3);
  border-radius: 8px;
  padding: 1rem;
  text-align: center;
}

.survival-item.danger {
  border-color: #ef4444;
  background: rgba(239, 68, 68, 0.1);
}

.survival-cmd {
  font-family: 'Fira Code', monospace;
  font-size: 1.25rem;
  font-weight: 700;
  color: #fbbf24;
  margin-bottom: 0.5rem;
}

.survival-desc {
  font-size: 0.85rem;
  color: #cbd5e1;
}

.use-case-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.use-case-item {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.use-case-scenario {
  display: flex;
  align-items: center;
  gap: 12px;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.05rem;
}

.use-case-solution {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

.use-case-tool {
  background: rgba(59, 130, 246, 0.1);
  border-radius: 6px;
  padding: 0.5rem;
  font-size: 0.85rem;
  color: #60a5fa;
  text-align: center;
}

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

.summary-row.header {
  background: rgba(59, 130, 246, 0.1);
  font-weight: 700;
  color: #60a5fa;
}

.summary-row div {
  display: flex;
  align-items: center;
  color: #e2e8f0;
  font-size: 0.9rem;
}

@media (max-width: 768px) {
  .remote-diagram {
    flex-direction: column;
  }

  .summary-row {
    grid-template-columns: 1fr;
    gap: 0.5rem;
  }
}
</style>
