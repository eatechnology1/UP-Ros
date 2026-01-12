<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      YAML (YAML Ain't Markup Language) es el formato de configuración más legible para humanos. En
      ROS 2, es el <strong>estándar de facto</strong> para archivos de parámetros, configuración de
      nodos, y launch files. <br /><br />
      A diferencia de XML (verboso) y JSON (estricto), YAML prioriza la
      <strong>simplicidad</strong> y la <strong>legibilidad</strong>.
    </TextBlock>

    <AlertBlock type="info" title="¿Cuándo usar YAML en ROS 2?">
      <strong>Parámetros:</strong> Configuración de nodos sin recompilar
      <br />
      <strong>Launch files:</strong> Alternativa más legible que XML
      <br />
      <strong>Configuración:</strong> Nav2, SLAM, controladores
      <br />
      <strong>Mapas:</strong> Metadatos de mapas de navegación <br /><br />
      YAML es <strong>humano primero</strong>: fácil de leer, fácil de editar.
    </AlertBlock>

    <!-- ANATOMÍA -->
    <div class="section-group">
      <SectionTitle>1. Anatomía: La Tiranía del Espacio</SectionTitle>

      <div class="anatomy-demo">
        <div class="anatomy-visual">
          <div class="yaml-structure">
            <div class="yaml-line level-0">
              <span class="key">robot_config</span><span class="colon">:</span>
            </div>
            <div class="yaml-line level-1">
              <span class="indent">··</span>
              <span class="key">nombre</span><span class="colon">:</span>
              <span class="value string">TurtleBot</span>
            </div>
            <div class="yaml-line level-1">
              <span class="indent">··</span>
              <span class="key">sensores</span><span class="colon">:</span>
            </div>
            <div class="yaml-line level-2">
              <span class="indent">····</span>
              <span class="dash">-</span>
              <span class="value string">lidar</span>
            </div>
            <div class="yaml-line level-2">
              <span class="indent">····</span>
              <span class="dash">-</span>
              <span class="value string">camara</span>
            </div>
          </div>

          <div class="annotations">
            <div class="annotation indent-ann">
              <q-icon name="space_bar" />
              <span>Indentación (2 espacios)</span>
            </div>
            <div class="annotation dash-ann">
              <q-icon name="list" />
              <span>Lista (guión -)</span>
            </div>
            <div class="annotation colon-ann">
              <q-icon name="more_vert" />
              <span>Clave: Valor</span>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Reglas de Oro</SectionTitle>
        <div class="rules-grid">
          <div class="rule-item">
            <q-icon name="space_bar" color="blue-4" size="md" />
            <div class="rule-text">
              <strong>Solo espacios</strong>
              <br />
              ❌ Tabs prohibidos | ✅ 2 espacios por nivel
            </div>
          </div>
          <div class="rule-item">
            <q-icon name="format_indent_increase" color="purple-4" size="md" />
            <div class="rule-text">
              <strong>Indentación consistente</strong>
              <br />
              Todos los hijos al mismo nivel
            </div>
          </div>
          <div class="rule-item">
            <q-icon name="more_horiz" color="green-4" size="md" />
            <div class="rule-text">
              <strong>Espacio después de :</strong>
              <br />
              <code>key: value</code> ✅ | <code>key:value</code> ❌
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="danger" title="💀 Muerte por Tabulador">
          Si mezclas espacios y tabs, YAML lanzará un error críptico. Configura tu editor (VS Code)
          para convertir tabs en espacios automáticamente:
          <br /><br />
          <code>Settings → Editor: Insert Spaces → ✅</code>
        </AlertBlock>
      </div>
    </div>

    <!-- TIPOS DE DATOS -->
    <div class="section-group">
      <SectionTitle>2. Tipos de Datos</SectionTitle>

      <div class="types-grid">
        <div class="type-card string">
          <div class="type-header">
            <q-icon name="text_fields" size="lg" />
            <span>String (Texto)</span>
          </div>
          <CodeBlock
            lang="yaml"
            content="nombre: TurtleBot
descripcion: 'Robot móvil'
path: &quot;/home/user/robot&quot;"
          />
          <div class="type-note">Comillas opcionales (salvo caracteres especiales)</div>
        </div>

        <div class="type-card number">
          <div class="type-header">
            <q-icon name="123" size="lg" />
            <span>Number (Número)</span>
          </div>
          <CodeBlock
            lang="yaml"
            content="velocidad: 2.5
puerto: 8080
temperatura: -15.3"
          />
          <div class="type-note">Enteros o decimales (sin comillas)</div>
        </div>

        <div class="type-card boolean">
          <div class="type-header">
            <q-icon name="toggle_on" size="lg" />
            <span>Boolean</span>
          </div>
          <CodeBlock
            lang="yaml"
            content="activo: true
debug: false
# También: yes/no, on/off"
          />
          <div class="type-note">Minúsculas o mayúsculas</div>
        </div>

        <div class="type-card null">
          <div class="type-header">
            <q-icon name="block" size="lg" />
            <span>Null (Vacío)</span>
          </div>
          <CodeBlock
            lang="yaml"
            content="error: null
# También: ~
dato: ~"
          />
          <div class="type-note">Representa ausencia de valor</div>
        </div>

        <div class="type-card array">
          <div class="type-header">
            <q-icon name="list" size="lg" />
            <span>Lista (Array)</span>
          </div>
          <CodeBlock
            lang="yaml"
            content="# Estilo bloque
sensores:
  - lidar
  - camara

# Estilo inline
posicion: [1.0, 2.5, 0.0]"
          />
          <div class="type-note">Dos formas: bloque o inline</div>
        </div>

        <div class="type-card object">
          <div class="type-header">
            <q-icon name="data_object" size="lg" />
            <span>Objeto (Mapa)</span>
          </div>
          <CodeBlock
            lang="yaml"
            content="bateria:
  voltaje: 12.4
  nivel: 85
  estado: cargando"
          />
          <div class="type-note">Pares clave-valor anidados</div>
        </div>
      </div>
    </div>

    <!-- PARÁMETROS ROS 2 -->
    <div class="section-group">
      <SectionTitle>3. Parámetros ROS 2: La Estructura Mágica</SectionTitle>
      <TextBlock>
        En ROS 2, los parámetros siguen una estructura específica:
        <code>nombre_nodo → ros__parameters → tus_variables</code>
      </TextBlock>

      <CodeBlock
        title="robot_params.yaml"
        lang="yaml"
        content="# Estructura obligatoria para ROS 2
robot_controller:
  ros__parameters:  # Doble guion bajo
    max_speed: 2.0
    min_speed: 0.1
    use_sim_time: true

    # Objeto anidado
    pid_config:
      kp: 1.0
      ki: 0.1
      kd: 0.05

    # Lista
    waypoints:
      - [0.0, 0.0]
      - [1.0, 2.0]
      - [3.0, 1.5]

# Múltiples nodos en un archivo
navigation:
  ros__parameters:
    robot_radius: 0.3
    inflation_radius: 0.5"
        :copyable="true"
      />

      <div class="q-mt-md">
        <AlertBlock type="warning" title="¡Cuidado con ros__parameters!">
          El nombre <code>ros__parameters</code> usa <strong>doble guion bajo</strong> (__), no
          guion simple. Este es el error #1 en archivos YAML de ROS 2.
        </AlertBlock>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Cargar Parámetros</SectionTitle>
        <div class="row q-col-gutter-md">
          <div class="col-12 col-md-6">
            <div class="param-card">
              <div class="param-header">
                <q-icon name="terminal" size="md" />
                <span>Desde Terminal</span>
              </div>
              <CodeBlock
                lang="bash"
                content="ros2 run mi_paquete mi_nodo \\
  --ros-args \\
  --params-file robot_params.yaml"
                :copyable="true"
              />
            </div>
          </div>

          <div class="col-12 col-md-6">
            <div class="param-card">
              <div class="param-header">
                <q-icon name="rocket_launch" size="md" />
                <span>Desde Launch File</span>
              </div>
              <CodeBlock
                lang="python"
                content="Node(
    package='mi_paquete',
    executable='mi_nodo',
    parameters=[robot_params_path]
)"
                :copyable="true"
              />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ANCLAS Y ALIAS -->
    <div class="section-group">
      <SectionTitle>4. Anclas y Alias: No te Repitas (DRY)</SectionTitle>
      <TextBlock>
        YAML permite reutilizar configuraciones con <strong>anclas (&)</strong> y
        <strong>alias (*)</strong>. Perfecto para robots con múltiples sensores o ruedas idénticas.
      </TextBlock>

      <div class="anchor-demo q-mt-md">
        <div class="anchor-section define">
          <div class="anchor-label">
            <q-icon name="anchor" color="blue-4" />
            <span>1. Definir Ancla (&)</span>
          </div>
          <CodeBlock
            lang="yaml"
            content="# Definir configuración base
config_rueda: &rueda_base
  radio: 0.15
  friccion: 0.8
  pid:
    kp: 10.0
    ki: 0.1
    kd: 0.5"
            :copyable="true"
          />
        </div>

        <div class="anchor-arrow">
          <q-icon name="arrow_forward" size="2rem" color="yellow-6" />
        </div>

        <div class="anchor-section use">
          <div class="anchor-label">
            <q-icon name="content_copy" color="green-4" />
            <span>2. Reutilizar Alias (*)</span>
          </div>
          <CodeBlock
            lang="yaml"
            content="rueda_frontal_izq:
  <<: *rueda_base  # Copia todo

rueda_frontal_der:
  <<: *rueda_base

rueda_trasera_izq:
  <<: *rueda_base
  radio: 0.20  # Override"
            :copyable="true"
          />
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="success" title="Ventajas">
          ✅ Cambias una vez, se aplica a todos
          <br />
          ✅ Menos errores de copy-paste
          <br />
          ✅ Archivos más cortos y mantenibles
        </AlertBlock>
      </div>
    </div>

    <!-- STRINGS MULTILÍNEA -->
    <div class="section-group">
      <SectionTitle>5. Strings Multilínea: El Poder Oculto</SectionTitle>
      <TextBlock>
        YAML tiene una característica única: <strong>strings multilínea</strong>. Perfecto para
        descripciones largas, comandos, o mensajes de error sin escapar caracteres.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-md">
        <div class="col-12 col-md-6">
          <div class="multiline-card">
            <div class="multiline-header">
              <q-icon name="format_align_left" size="md" />
              <span>Literal Block (|)</span>
            </div>
            <CodeBlock
              lang="yaml"
              content="descripcion: |
  Este robot es un TurtleBot3.
  Tiene 2 ruedas y un LIDAR.
  Velocidad máxima: 0.26 m/s.

  Líneas en blanco se preservan."
              :copyable="true"
            />
            <div class="multiline-note">
              <strong>Uso:</strong> Preserva saltos de línea y espacios. Ideal para comandos bash,
              mensajes largos.
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="multiline-card">
            <div class="multiline-header">
              <q-icon name="wrap_text" size="md" />
              <span>Folded Block (>)</span>
            </div>
            <CodeBlock
              lang="yaml"
              content="descripcion: >
  Este robot es un TurtleBot3.
  Tiene 2 ruedas y un LIDAR.
  Velocidad máxima: 0.26 m/s.

  Todo se une en una línea."
              :copyable="true"
            />
            <div class="multiline-note">
              <strong>Uso:</strong> Une líneas en un solo párrafo. Ideal para descripciones legibles
              en el archivo.
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Caso Práctico: Launch File con Comandos</SectionTitle>
        <CodeBlock
          title="launch_params.yaml"
          lang="yaml"
          content="robot_bringup:
  ros__parameters:
    # Comando de inicialización (multilínea)
    startup_command: |
      source /opt/ros/humble/setup.bash
      source ~/robot_ws/install/setup.bash
      ros2 launch robot_bringup robot.launch.py

    # Descripción legible
    robot_description: >
      TurtleBot3 Burger es un robot móvil diferencial
      diseñado para educación e investigación.
      Incluye LIDAR, IMU y encoders en las ruedas."
          :copyable="true"
        />
      </div>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>6. Errores Comunes</SectionTitle>

      <div class="errors-grid">
        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="red-4" size="lg" />
            <span>Tabs en lugar de espacios</span>
          </div>
          <div class="error-example">
            <div class="error-code wrong">
              <code>robot:</code>
              <br />
              <code>→→nombre: Bot</code>
              <br />
              <span class="error-mark">❌ Tab invisible</span>
            </div>
            <div class="error-arrow">→</div>
            <div class="error-code correct">
              <code>robot:</code>
              <br />
              <code>··nombre: Bot</code>
              <br />
              <span class="success-mark">✅ 2 espacios</span>
            </div>
          </div>
        </div>

        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="orange-4" size="lg" />
            <span>Sin espacio después de :</span>
          </div>
          <div class="error-example">
            <div class="error-code wrong">
              <code>velocidad:2.5</code>
              <br />
              <span class="error-mark">❌ Sin espacio</span>
            </div>
            <div class="error-arrow">→</div>
            <div class="error-code correct">
              <code>velocidad: 2.5</code>
              <br />
              <span class="success-mark">✅ Con espacio</span>
            </div>
          </div>
        </div>

        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="yellow-6" size="lg" />
            <span>Indentación inconsistente</span>
          </div>
          <div class="error-example">
            <div class="error-code wrong">
              <code>robot:</code>
              <br />
              <code>··nombre: Bot</code>
              <br />
              <code>····velocidad: 2.0</code>
              <br />
              <span class="error-mark">❌ Niveles mezclados</span>
            </div>
            <div class="error-arrow">→</div>
            <div class="error-code correct">
              <code>robot:</code>
              <br />
              <code>··nombre: Bot</code>
              <br />
              <code>··velocidad: 2.0</code>
              <br />
              <span class="success-mark">✅ Mismo nivel</span>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- VALIDACIÓN -->
    <div class="section-group">
      <SectionTitle>7. Herramientas de Validación</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-6">
          <div class="tool-card">
            <div class="tool-header">
              <q-icon name="check_circle" color="green-4" />
              <span>Validar en Python</span>
            </div>
            <CodeBlock
              lang="python"
              content="import yaml

with open('params.yaml', 'r') as f:
    try:
        data = yaml.safe_load(f)
        print('YAML valido')
    except yaml.YAMLError as e:
        print(f'Error: {e}')"
              :copyable="true"
            />
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="tool-card">
            <div class="tool-header">
              <q-icon name="terminal" color="blue-4" />
              <span>Validar en Terminal</span>
            </div>
            <CodeBlock
              lang="bash"
              content="# Instalar yamllint
pip install yamllint

# Validar sintaxis
yamllint params.yaml

# Validar con ROS 2
ros2 param load /mi_nodo params.yaml"
              :copyable="true"
            />
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="info" title="Validadores Online">
          <strong>YAML Lint:</strong> http://www.yamllint.com/
          <br />
          <strong>YAML Validator:</strong> https://codebeautify.org/yaml-validator
          <br />
          <strong>YAML to JSON:</strong> https://onlineyamltools.com/convert-yaml-to-json
        </AlertBlock>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://youtu.be/Romc22GgusU"
            title="YAML en ROS 2"
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
      <SectionTitle>📝 Resumen</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>key: value</code>
          <span>Estructura básica</span>
        </div>
        <div class="summary-item">
          <code>2 espacios</code>
          <span>Indentación estándar</span>
        </div>
        <div class="summary-item">
          <code>- item</code>
          <span>Lista</span>
        </div>
        <div class="summary-item">
          <code>&anchor</code>
          <span>Definir ancla</span>
        </div>
        <div class="summary-item">
          <code>*alias</code>
          <span>Reutilizar ancla</span>
        </div>
        <div class="summary-item">
          <code>ros__parameters</code>
          <span>Parámetros ROS 2</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de YAML Válido" class="q-mt-lg">
        ✅ Solo espacios (no tabs)
        <br />
        ✅ Indentación consistente (2 espacios)
        <br />
        ✅ Espacio después de <code>:</code>
        <br />
        ✅ <code>ros__parameters</code> con doble guion bajo
        <br />
        ✅ Comentarios con <code>#</code>
        <br />
        ✅ Validado con <code>yamllint</code> o <code>yaml.safe_load()</code>
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

/* ANATOMY DEMO */
.anatomy-demo {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 3rem 2rem;
  margin-top: 1.5rem;
}

.anatomy-visual {
  display: flex;
  flex-direction: column;
  gap: 3rem;
}

.yaml-structure {
  font-family: 'Fira Code', monospace;
  font-size: 1.3rem;
}

.yaml-line {
  display: flex;
  align-items: center;
  padding: 0.25rem 0;
}

.indent {
  color: rgba(255, 255, 255, 0.15);
  user-select: none;
}

.key {
  color: #60a5fa;
  font-weight: 700;
}

.colon {
  color: #64748b;
  margin: 0 0.5rem;
}

.value.string {
  color: #22c55e;
}

.dash {
  color: #f97316;
  margin-right: 0.5rem;
}

.annotations {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 2rem;
}

.annotation {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  font-size: 0.9rem;
  font-weight: 700;
}

.indent-ann {
  color: #60a5fa;
}

.dash-ann {
  color: #f97316;
}

.colon-ann {
  color: #a855f7;
}

/* RULES GRID */
.rules-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1.5rem;
  margin-top: 1rem;
}

.rule-item {
  display: flex;
  gap: 1rem;
  align-items: flex-start;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.rule-text {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.rule-text code {
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
}

/* TYPES GRID */
.types-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.type-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  display: flex;
  flex-direction: column;
  height: 100%;
}

.type-card.string {
  border-top: 4px solid #22c55e;
}

.type-card.number {
  border-top: 4px solid #60a5fa;
}

.type-card.boolean {
  border-top: 4px solid #a855f7;
}

.type-card.null {
  border-top: 4px solid #64748b;
}

.type-card.array {
  border-top: 4px solid #f97316;
}

.type-card.object {
  border-top: 4px solid #fbbf24;
}

.type-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
  flex-shrink: 0;
}

.type-note {
  padding: 1rem 1.5rem;
  font-size: 0.85rem;
  color: #94a3b8;
  margin-top: auto;
}

/* PARAM CARDS */
.param-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
  display: flex;
  flex-direction: column;
}

.param-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
  flex-shrink: 0;
}

/* ANCHOR DEMO */
.anchor-demo {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.anchor-section {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.anchor-label {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  font-size: 1.05rem;
  color: #f1f5f9;
}

.anchor-arrow {
  color: #fbbf24;
}

/* MULTILINE CARDS */
.multiline-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
  display: flex;
  flex-direction: column;
}

.multiline-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
  flex-shrink: 0;
}

.multiline-note {
  padding: 1rem 1.5rem;
  font-size: 0.85rem;
  color: #94a3b8;
  margin-top: auto;
}

/* ERRORS GRID */
.errors-grid {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.error-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.error-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
}

.error-example {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 1rem;
  padding: 1.5rem;
  align-items: center;
}

.error-code {
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
  padding: 1rem;
  border-radius: 8px;
}

.error-code.wrong {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid rgba(239, 68, 68, 0.3);
}

.error-code.correct {
  background: rgba(34, 197, 94, 0.1);
  border: 1px solid rgba(34, 197, 94, 0.3);
}

.error-mark {
  color: #fca5a5;
  font-size: 0.85rem;
  margin-left: 0.5rem;
}

.success-mark {
  color: #86efac;
  font-size: 0.85rem;
  margin-left: 0.5rem;
}

.error-arrow {
  color: #fbbf24;
  font-size: 1.5rem;
}

/* TOOL CARD */
.tool-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
  display: flex;
  flex-direction: column;
}

.tool-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
  flex-shrink: 0;
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
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
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
  .annotations {
    grid-template-columns: 1fr;
  }

  .anchor-demo {
    grid-template-columns: 1fr;
  }

  .anchor-arrow {
    transform: rotate(90deg);
  }

  .error-example {
    grid-template-columns: 1fr;
  }

  .error-arrow {
    transform: rotate(90deg);
  }
}
</style>
