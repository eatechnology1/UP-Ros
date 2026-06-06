<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      JSON (JavaScript Object Notation) es el formato de intercambio de datos más usado en la web.
      En ROS 2, lo usarás para comunicarte con interfaces web, configurar parámetros, y trabajar con
      APIs REST.
      <br /><br />
      A diferencia de XML, JSON es <strong>minimalista</strong>: sin etiquetas de cierre, sin
      atributos, solo pares clave-valor.
    </TextBlock>

    <AlertBlock type="info" title="¿Cuándo usar JSON en ROS 2?">
      <strong>rosbridge:</strong> Comunicación web (WebSockets)
      <br />
      <strong>REST APIs:</strong> Servicios HTTP para control remoto
      <br />
      <strong>Configuración:</strong> Archivos de parámetros (alternativa a YAML)
      <br />
      <strong>Logging:</strong> Exportar datos de sensores <br /><br />
      JSON es <strong>universal</strong>: cualquier lenguaje puede leerlo.
    </AlertBlock>

    <!-- ANATOMÍA -->
    <div class="section-group">
      <SectionTitle>1. Anatomía: Clave-Valor</SectionTitle>

      <div class="anatomy-demo">
        <div class="anatomy-visual">
          <div class="json-breakdown">
            <span class="bracket">{</span>
            <br />
            <span class="indent"></span>
            <span class="key">"nombre"</span>
            <span class="colon">:</span>
            <span class="value string">"TurtleBot"</span>
            <span class="comma">,</span>
            <br />
            <span class="indent"></span>
            <span class="key">"velocidad"</span>
            <span class="colon">:</span>
            <span class="value number">2.5</span>
            <br />
            <span class="bracket">}</span>
          </div>

          <div class="annotations">
            <div class="annotation key-ann">
              <q-icon name="vpn_key" />
              <span>Clave (siempre string)</span>
            </div>
            <div class="annotation value-ann">
              <q-icon name="data_object" />
              <span>Valor (varios tipos)</span>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Reglas de Sintaxis</SectionTitle>
        <div class="rules-grid">
          <div class="rule-item">
            <q-icon name="format_quote" color="yellow-6" size="md" />
            <div class="rule-text">
              <strong>Comillas dobles</strong>
              <br />
              <code>"clave"</code> ✅ | <code>'clave'</code> ❌
            </div>
          </div>
          <div class="rule-item">
            <q-icon name="more_horiz" color="blue-4" size="md" />
            <div class="rule-text">
              <strong>Sin coma final</strong>
              <br />
              <code>{"a": 1}</code> ✅ | <code>{"a": 1,}</code> ❌
            </div>
          </div>
          <div class="rule-item">
            <q-icon name="code" color="green-4" size="md" />
            <div class="rule-text">
              <strong>Sin comentarios</strong>
              <br />
              JSON puro no soporta <code>//</code> ni <code>/* */</code>
            </div>
          </div>
        </div>
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
          <CodeBlock lang="json" content='"nombre": "Robot1"' />
          <div class="type-note">Siempre entre comillas dobles</div>
        </div>

        <div class="type-card number">
          <div class="type-header">
            <q-icon name="123" size="lg" />
            <span>Number (Número)</span>
          </div>
          <CodeBlock lang="json" content='"velocidad": 2.5' />
          <div class="type-note">Enteros o decimales (sin comillas)</div>
        </div>

        <div class="type-card boolean">
          <div class="type-header">
            <q-icon name="toggle_on" size="lg" />
            <span>Boolean</span>
          </div>
          <CodeBlock lang="json" content='"activo": true' />
          <div class="type-note"><code>true</code> o <code>false</code> (minúsculas)</div>
        </div>

        <div class="type-card null">
          <div class="type-header">
            <q-icon name="block" size="lg" />
            <span>Null (Vacío)</span>
          </div>
          <CodeBlock lang="json" content='"error": null' />
          <div class="type-note">Representa ausencia de valor</div>
        </div>

        <div class="type-card array">
          <div class="type-header">
            <q-icon name="list" size="lg" />
            <span>Array (Lista)</span>
          </div>
          <CodeBlock lang="json" content='"posicion": [1.0, 2.5, 0.0]' />
          <div class="type-note">Lista ordenada de valores</div>
        </div>

        <div class="type-card object">
          <div class="type-header">
            <q-icon name="data_object" size="lg" />
            <span>Object (Objeto)</span>
          </div>
          <CodeBlock
            lang="json"
            content='"bateria": {
  "voltaje": 12.4,
  "nivel": 85
}'
          />
          <div class="type-note">Pares clave-valor anidados</div>
        </div>
      </div>
    </div>

    <!-- ANIDACIÓN -->
    <div class="section-group">
      <SectionTitle>3. Estructuras Anidadas</SectionTitle>
      <TextBlock>
        JSON permite anidar objetos y arrays infinitamente. Esto es perfecto para representar robots
        con múltiples sensores y actuadores.
      </TextBlock>

      <CodeBlock
        title="robot_config.json"
        lang="json"
        content='{
  "robot": {
    "id": "robot_001",
    "tipo": "movil",
    "sensores": [
      {
        "nombre": "lidar",
        "modelo": "RPLidar A1",
        "rango_max": 12.0,
        "activo": true
      },
      {
        "nombre": "camara",
        "modelo": "Intel RealSense",
        "resolucion": [1920, 1080],
        "fps": 30
      }
    ],
    "motores": {
      "izquierdo": {
        "velocidad_max": 3.0,
        "encoder_ticks": 1024
      },
      "derecho": {
        "velocidad_max": 3.0,
        "encoder_ticks": 1024
      }
    }
  }
}'
        :copyable="true"
      />
    </div>

    <!-- ROSBRIDGE -->
    <div class="section-group">
      <SectionTitle>4. Caso Práctico: rosbridge (Web ↔ ROS 2)</SectionTitle>
      <TextBlock>
        <strong>rosbridge</strong> es un paquete que traduce mensajes ROS 2 a JSON para que puedas
        controlar robots desde una página web usando WebSockets.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-md">
        <div class="col-12 col-md-6">
          <div class="rosbridge-card publish">
            <div class="rosbridge-header">
              <q-icon name="publish" size="md" />
              <span>Publicar (Web → Robot)</span>
            </div>
            <CodeBlock
              lang="json"
              content='{
  "op": "publish",
  "topic": "/cmd_vel",
  "msg": {
    "linear": {
      "x": 0.5,
      "y": 0.0,
      "z": 0.0
    },
    "angular": {
      "x": 0.0,
      "y": 0.0,
      "z": 1.2
    }
  }
}'
              :copyable="true"
            />
            <div class="rosbridge-note">
              Envía comando de velocidad: avanzar 0.5 m/s y girar 1.2 rad/s
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="rosbridge-card subscribe">
            <div class="rosbridge-header">
              <q-icon name="sensors" size="md" />
              <span>Suscribirse (Robot → Web)</span>
            </div>
            <CodeBlock
              lang="json"
              content='{
  "op": "subscribe",
  "topic": "/scan",
  "type": "sensor_msgs/LaserScan",
  "throttle_rate": 100
}'
              :copyable="true"
            />
            <div class="rosbridge-note">Recibe datos del LIDAR a 100ms (10 Hz)</div>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="success" title="Instalación de rosbridge">
          <CodeBlock
            lang="bash"
            content="# Instalar rosbridge
sudo apt install ros-humble-rosbridge-suite

# Lanzar servidor WebSocket
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Conectar desde JavaScript
const ws = new WebSocket('ws://localhost:9090');"
            :copyable="true"
          />
        </AlertBlock>
      </div>
    </div>

    <!-- JSON vs PYTHON -->
    <div class="section-group">
      <SectionTitle>5. JSON vs Python Dict: Las Diferencias</SectionTitle>
      <TextBlock>
        JSON y los diccionarios de Python se ven similares, pero <strong>NO son lo mismo</strong>.
        Aquí están las diferencias críticas:
      </TextBlock>

      <div class="comparison-table q-mt-md">
        <div class="comparison-row header">
          <div class="comparison-cell">Característica</div>
          <div class="comparison-cell">Python Dict</div>
          <div class="comparison-cell">JSON</div>
        </div>
        <div class="comparison-row">
          <div class="comparison-cell">Comillas</div>
          <div class="comparison-cell"><code>'simple'</code> o <code>"doble"</code></div>
          <div class="comparison-cell">Solo <code>"doble"</code></div>
        </div>
        <div class="comparison-row">
          <div class="comparison-cell">Booleanos</div>
          <div class="comparison-cell"><code>True</code>, <code>False</code></div>
          <div class="comparison-cell"><code>true</code>, <code>false</code></div>
        </div>
        <div class="comparison-row">
          <div class="comparison-cell">Null/None</div>
          <div class="comparison-cell"><code>None</code></div>
          <div class="comparison-cell"><code>null</code></div>
        </div>
        <div class="comparison-row">
          <div class="comparison-cell">Coma final</div>
          <div class="comparison-cell">Permitida</div>
          <div class="comparison-cell">❌ Prohibida</div>
        </div>
        <div class="comparison-row">
          <div class="comparison-cell">Comentarios</div>
          <div class="comparison-cell"><code>#</code> permitido</div>
          <div class="comparison-cell">❌ No soportado</div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Conversión en Python</SectionTitle>
        <CodeBlock
          title="Trabajar con JSON en Python"
          lang="python"
          content="import json

# Python dict to JSON string
robot_dict = {
    'nombre': 'TurtleBot',
    'activo': True,
    'sensores': None
}
json_string = json.dumps(robot_dict)
print(json_string)

# JSON string to Python dict
json_data = '{&quot;velocidad&quot;: 2.5}'
robot_dict = json.loads(json_data)
print(robot_dict['velocidad'])

# Leer/escribir archivos JSON
with open('config.json', 'w') as f:
with open('config.json', 'r') as f:
    data = json.load(f)"
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
            <span>Coma final (Trailing Comma)</span>
          </div>
          <div class="error-example">
            <div class="error-code wrong">
              <code>{</code>
              <br />
              <code>&nbsp;&nbsp;"id": 1,</code>
              <br />
              <code>&nbsp;&nbsp;"nombre": "Robot",</code>
              <span class="error-mark">← ❌ Coma extra</span>
              <br />
              <code>}</code>
            </div>
            <div class="error-arrow">→</div>
            <div class="error-code correct">
              <code>{</code>
              <br />
              <code>&nbsp;&nbsp;"id": 1,</code>
              <br />
              <code>&nbsp;&nbsp;"nombre": "Robot"</code>
              <span class="success-mark">← ✅ Sin coma</span>
              <br />
              <code>}</code>
            </div>
          </div>
        </div>

        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="orange-4" size="lg" />
            <span>Comillas simples</span>
          </div>
          <div class="error-example">
            <div class="error-code wrong">
              <code>{'nombre': 'Robot'}</code>
              <br />
              <span class="error-mark">❌ Comillas simples</span>
            </div>
            <div class="error-arrow">→</div>
            <div class="error-code correct">
              <code>{"nombre": "Robot"}</code>
              <br />
              <span class="success-mark">✅ Comillas dobles</span>
            </div>
          </div>
        </div>

        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="yellow-6" size="lg" />
            <span>Booleanos con mayúscula</span>
          </div>
          <div class="error-example">
            <div class="error-code wrong">
              <code>{"activo": True}</code>
              <br />
              <span class="error-mark">❌ Python style</span>
            </div>
            <div class="error-arrow">→</div>
            <div class="error-code correct">
              <code>{"activo": true}</code>
              <br />
              <span class="success-mark">✅ JSON style</span>
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
              content="import json

json_string = '{&quot;nombre&quot;: &quot;Robot&quot;}'

try:
    data = json.loads(json_string)
    print('JSON valido')
except json.JSONDecodeError as e:
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
              content="# Instalar jq (JSON processor)
sudo apt install jq

# Validar sintaxis
cat config.json | jq .

# Pretty print
jq . config.json

# Extraer valor
jq '.robot.nombre' config.json"
              :copyable="true"
            />
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="info" title="Validadores Online">
          <strong>JSONLint:</strong> https://jsonlint.com/
          <br />
          <strong>JSON Formatter:</strong> https://jsonformatter.org/
          <br />
          <strong>JSON Editor Online:</strong> https://jsoneditoronline.org/
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
            title="JSON en ROS 2"
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
          <code>{"clave": "valor"}</code>
          <span>Estructura básica</span>
        </div>
        <div class="summary-item">
          <code>"string"</code>
          <span>Comillas dobles obligatorias</span>
        </div>
        <div class="summary-item">
          <code>true/false/null</code>
          <span>Minúsculas</span>
        </div>
        <div class="summary-item">
          <code>[1, 2, 3]</code>
          <span>Arrays</span>
        </div>
        <div class="summary-item">
          <code>rosbridge</code>
          <span>Web ↔ ROS 2</span>
        </div>
        <div class="summary-item">
          <code>json.loads()</code>
          <span>String → Dict</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de JSON Válido" class="q-mt-lg">
        ✅ Comillas dobles en claves y strings
        <br />
        ✅ Sin coma después del último elemento
        <br />
        ✅ Booleanos en minúsculas (<code>true</code>, <code>false</code>)
        <br />
        ✅ <code>null</code> en minúsculas
        <br />
        ✅ Sin comentarios
        <br />
        ✅ Validado con <code>jq</code> o <code>json.loads()</code>
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
  background: var(--bg-surface);
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

.json-breakdown {
  font-family: 'Fira Code', monospace;
  font-size: 1.5rem;
  text-align: center;
}

.bracket {
  color: #64748b;
  font-weight: 700;
}

.key {
  color: var(--text-warning, #d97706);
  font-weight: 700;
}

.colon {
  color: #64748b;
  margin: 0 0.5rem;
}

.value.string {
  color: var(--text-code);
}

.value.number {
  color: var(--text-info, #2563eb);
}

.comma {
  color: #64748b;
}

.indent {
  display: inline-block;
  width: 2rem;
}

.annotations {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
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

.key-ann {
  color: var(--text-warning, #d97706);
}

.value-ann {
  color: var(--text-info, #2563eb);
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
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.rule-text {
  color: var(--text-secondary);
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
  background: var(--bg-surface);
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
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: var(--text-primary);
  flex-shrink: 0;
}

.type-note {
  padding: 1rem 1.5rem;
  font-size: 0.85rem;
  color: var(--text-muted);
  margin-top: auto;
}

/* ROSBRIDGE CARDS */
.rosbridge-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
  display: flex;
  flex-direction: column;
}

.rosbridge-card.publish {
  border-top: 4px solid #3b82f6;
}

.rosbridge-card.subscribe {
  border-top: 4px solid #22c55e;
}

.rosbridge-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: var(--text-primary);
  flex-shrink: 0;
}

.rosbridge-note {
  padding: 1rem 1.5rem;
  background: rgba(59, 130, 246, 0.1);
  font-size: 0.85rem;
  color: #93c5fd;
  margin-top: auto;
}

/* COMPARISON TABLE */
.comparison-table {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.comparison-row {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
}

.comparison-row:last-child {
  border-bottom: none;
}

.comparison-row.header {
  background: var(--bg-surface-hover);
  font-weight: 700;
  color: var(--text-primary);
}

.comparison-cell {
  padding: 1rem 1.5rem;
  border-right: 1px solid rgba(148, 163, 184, 0.2);
  color: var(--text-secondary);
}

.comparison-cell:last-child {
  border-right: none;
}

.comparison-cell code {
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

/* ERRORS GRID */
.errors-grid {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.error-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.error-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: var(--text-primary);
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
  color: var(--text-danger, #dc2626);
  font-size: 0.85rem;
  margin-left: 0.5rem;
}

.success-mark {
  color: var(--text-code);
  font-size: 0.85rem;
  margin-left: 0.5rem;
}

.error-arrow {
  color: var(--text-warning, #d97706);
  font-size: 1.5rem;
}

/* TOOL CARD */
.tool-card {
  background: var(--bg-surface);
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
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: var(--text-primary);
  flex-shrink: 0;
}

/* VIDEO */
.video-container {
  background: var(--bg-surface);
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
  color: var(--text-muted);
  font-size: 0.85rem;
}

/* SUMMARY */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.summary-item code {
  font-family: 'Fira Code', monospace;
  color: var(--text-code);
  font-size: 1rem;
}

.summary-item span {
  color: var(--text-secondary);
  font-size: 0.85rem;
}

@media (max-width: 768px) {
  .annotations {
    grid-template-columns: 1fr;
  }

  .error-example {
    grid-template-columns: 1fr;
  }

  .error-arrow {
    transform: rotate(90deg);
  }

  .comparison-row {
    grid-template-columns: 1fr;
  }

  .comparison-cell {
    border-right: none;
    border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  }
}
</style>
