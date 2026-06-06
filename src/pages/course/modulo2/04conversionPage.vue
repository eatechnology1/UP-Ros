<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      En ROS 2, trabajarás con múltiples formatos simultáneamente: YAML para parámetros, JSON para
      APIs web, XML para URDF. Saber convertir entre ellos es esencial para integrar sistemas y
      depurar problemas.
      <br /><br />
      La clave: <strong>el diccionario de Python</strong> es el formato universal. Todo pasa por
      ahí.
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué convertir formatos?">
      <strong>Depuración:</strong> Ver parámetros YAML en formato JSON más legible
      <br />
      <strong>Integración:</strong> Convertir config YAML a JSON para APIs web
      <br />
      <strong>Migración:</strong> Pasar de XML (ROS 1) a YAML (ROS 2)
      <br />
      <strong>Validación:</strong> Comparar configuraciones en diferentes formatos
    </AlertBlock>

    <!-- EL HUB CENTRAL -->
    <div class="section-group">
      <SectionTitle>1. El Diccionario como Hub Central</SectionTitle>

      <div class="hub-diagram">
        <div class="hub-center">
          <div class="hub-circle">
            <q-icon name="data_object" size="3rem" color="white" />
            <div class="hub-label">Python Dict</div>
          </div>
        </div>

        <div class="hub-formats">
          <div class="format-node yaml">
            <q-icon name="settings" size="2rem" />
            <span>YAML</span>
          </div>
          <div class="format-node json">
            <q-icon name="code" size="2rem" />
            <span>JSON</span>
          </div>
          <div class="format-node xml">
            <q-icon name="description" size="2rem" />
            <span>XML</span>
          </div>
        </div>

        <div class="hub-arrows">
          <div class="arrow arrow-1"></div>
          <div class="arrow arrow-2"></div>
          <div class="arrow arrow-3"></div>
        </div>
      </div>

      <TextBlock class="q-mt-lg">
        No convertimos directamente YAML → JSON. Primero cargamos a un diccionario Python, luego
        exportamos al formato deseado. Esto nos da flexibilidad para manipular los datos en el
        medio.
      </TextBlock>
    </div>

    <!-- CONVERSIONES BÁSICAS -->
    <div class="section-group">
      <SectionTitle>2. Conversiones Básicas</SectionTitle>

      <div class="conversion-grid">
        <!-- YAML → Dict -->
        <div class="conversion-card">
          <div class="conversion-header yaml">
            <q-icon name="settings" />
            <span>YAML → Dict</span>
          </div>
          <CodeBlock
            lang="python"
            content="import yaml

with open('config.yaml', 'r') as f:
    data = yaml.safe_load(f)

# data es ahora un dict de Python
print(type(data))  # <class 'dict'>
print(data['robot']['velocidad'])"
            :copyable="true"
          />
        </div>

        <!-- Dict → YAML -->
        <div class="conversion-card">
          <div class="conversion-header yaml">
            <q-icon name="settings" />
            <span>Dict → YAML</span>
          </div>
          <CodeBlock
            lang="python"
            content="import yaml

data = {
    'robot': {
        'velocidad': 2.5,
        'sensores': ['lidar', 'camara']
    }
}

with open('output.yaml', 'w') as f:
    yaml.dump(data, f, default_flow_style=False)"
            :copyable="true"
          />
        </div>

        <!-- JSON → Dict -->
        <div class="conversion-card">
          <div class="conversion-header json">
            <q-icon name="code" />
            <span>JSON → Dict</span>
          </div>
          <CodeBlock
            lang="python"
            content="import json

with open('data.json', 'r') as f:
    data = json.load(f)

# data es ahora un dict de Python
print(data['nombre'])"
            :copyable="true"
          />
        </div>

        <!-- Dict → JSON -->
        <div class="conversion-card">
          <div class="conversion-header json">
            <q-icon name="code" />
            <span>Dict → JSON</span>
          </div>
          <CodeBlock
            lang="python"
            content="import json

data = {
    'robot': 'TurtleBot',
    'activo': True,
    'velocidad': 2.5
}

with open('output.json', 'w') as f:
    json.dump(data, f, indent=2)"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- CONVERSIÓN COMPLETA -->
    <div class="section-group">
      <SectionTitle>3. Pipeline Completo: YAML → JSON</SectionTitle>

      <CodeBlock
        title="convert_yaml_to_json.py"
        lang="python"
        content="#!/usr/bin/env python3
import yaml
import json
import sys

def yaml_to_json(yaml_file, json_file):
    # 1. Leer YAML
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    # 2. (Opcional) Manipular datos
    # Ejemplo: agregar timestamp
    from datetime import datetime
    data['converted_at'] = datetime.now().isoformat()

    # 3. Escribir JSON
    with open(json_file, 'w') as f:
        json.dump(data, f, indent=2)

    print(f'Convertido: {yaml_file} -> {json_file}')

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Uso: python convert.py input.yaml output.json')
        sys.exit(1)

    yaml_to_json(sys.argv[1], sys.argv[2])"
        :copyable="true"
      />

      <div class="q-mt-md">
        <AlertBlock type="success" title="Uso">
          <CodeBlock
            lang="bash"
            content="python convert_yaml_to_json.py robot_params.yaml robot_params.json"
            :copyable="true"
          />
        </AlertBlock>
      </div>
    </div>

    <!-- CASO PRÁCTICO ROS 2 -->
    <div class="section-group">
      <SectionTitle>4. Caso Práctico: Parámetros ROS 2 → JSON para Web</SectionTitle>
      <TextBlock>
        Escenario: Tienes parámetros de navegación en YAML y quieres exponerlos en una API REST para
        que una interfaz web los visualice.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-md">
        <div class="col-12 col-md-6">
          <div class="example-card input">
            <div class="example-header">
              <q-icon name="settings" size="md" />
              <span>Input: nav_params.yaml</span>
            </div>
            <CodeBlock
              lang="yaml"
              content="nav2_controller:
  ros__parameters:
    max_vel_x: 0.26
    min_vel_x: -0.26
    max_vel_theta: 1.0

    # PID gains
    xy_goal_tolerance: 0.25
    yaw_goal_tolerance: 0.25"
            />
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="example-card output">
            <div class="example-header">
              <q-icon name="code" size="md" />
              <span>Output: API Response (JSON)</span>
            </div>
            <CodeBlock
              lang="json"
              content='{
  "controller": "nav2_controller",
  "parameters": {
    "max_vel_x": 0.26,
    "min_vel_x": -0.26,
    "max_vel_theta": 1.0,
    "xy_goal_tolerance": 0.25,
    "yaw_goal_tolerance": 0.25
  }
}'
            />
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          title="Script de conversión"
          lang="python"
          content="import yaml
import json

# Leer parámetros ROS 2
with open('nav_params.yaml', 'r') as f:
    ros_params = yaml.safe_load(f)

# Extraer y reformatear
for node_name, config in ros_params.items():
    if 'ros__parameters' in config:
        api_response = {
            'controller': node_name,
            'parameters': config['ros__parameters']
        }

        # Guardar como JSON
        with open(f'{node_name}.json', 'w') as f:
            json.dump(api_response, f, indent=2)

        print(f'Exportado: {node_name}.json')"
          :copyable="true"
        />
      </div>
    </div>

    <!-- HERRAMIENTAS CLI -->
    <div class="section-group">
      <SectionTitle>5. Herramientas de Terminal</SectionTitle>

      <div class="tools-grid">
        <div class="tool-item jq">
          <div class="tool-icon">
            <div class="tool-badge">jq</div>
          </div>
          <div class="tool-content">
            <div class="tool-title">JSON Processor</div>
            <div class="tool-desc">
              Procesa, filtra y formatea JSON desde la terminal. Indispensable para depuración.
            </div>
            <CodeBlock
              lang="bash"
              content="# Instalar
sudo apt install jq

# Pretty print
cat data.json | jq .

# Filtrar valor
jq '.robot.velocidad' data.json

# Convertir YAML a JSON (con yq)
yq eval -o=json config.yaml | jq ."
              :copyable="true"
            />
          </div>
        </div>

        <div class="tool-item yq">
          <div class="tool-icon">
            <div class="tool-badge">yq</div>
          </div>
          <div class="tool-content">
            <div class="tool-title">YAML Processor</div>
            <div class="tool-desc">
              Como jq pero para YAML. Permite convertir entre formatos directamente.
            </div>
            <CodeBlock
              lang="bash"
              content="# Instalar
pip install yq

# YAML a JSON
yq eval -o=json config.yaml > config.json

# JSON a YAML
yq eval -P data.json > data.yaml

# Filtrar en YAML
yq '.robot.velocidad' config.yaml"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- TABLA COMPARATIVA -->
    <div class="section-group">
      <SectionTitle>6. Comparación de Formatos</SectionTitle>
      <TextBlock>
        Cada formato tiene sus fortalezas. Aquí está la comparación lado a lado para ayudarte a
        elegir el correcto para cada situación en ROS 2.
      </TextBlock>

      <div class="comparison-table q-mt-md">
        <div class="comparison-row header">
          <div class="comparison-cell">Característica</div>
          <div class="comparison-cell">YAML</div>
          <div class="comparison-cell">XML</div>
          <div class="comparison-cell">JSON</div>
        </div>
        <div class="comparison-row">
          <div class="comparison-cell">Legibilidad</div>
          <div class="comparison-cell">⭐⭐⭐⭐⭐</div>
          <div class="comparison-cell">⭐⭐</div>
          <div class="comparison-cell">⭐⭐⭐</div>
        </div>
        <div class="comparison-row">
          <div class="comparison-cell">Verbosidad</div>
          <div class="comparison-cell">Mínima</div>
          <div class="comparison-cell">Máxima</div>
          <div class="comparison-cell">Media</div>
        </div>
        <div class="comparison-row">
          <div class="comparison-cell">Comentarios</div>
          <div class="comparison-cell">✅ <code>#</code></div>
          <div class="comparison-cell">✅ <code>&lt;!-- --&gt;</code></div>
          <div class="comparison-cell">❌</div>
        </div>
        <div class="comparison-row">
          <div class="comparison-cell">Anclas/Alias</div>
          <div class="comparison-cell">✅</div>
          <div class="comparison-cell">❌</div>
          <div class="comparison-cell">❌</div>
        </div>
        <div class="comparison-row">
          <div class="comparison-cell">Uso en ROS 2</div>
          <div class="comparison-cell">Parámetros, Launch</div>
          <div class="comparison-cell">URDF, Launch</div>
          <div class="comparison-cell">rosbridge, APIs</div>
        </div>
      </div>
    </div>

    <!-- COMPARACIÓN -->
    <div class="section-group">
      <SectionTitle>7. Mismo Dato, Tres Formatos</SectionTitle>

      <div class="format-comparison">
        <div class="format-example yaml">
          <div class="format-label">
            <q-icon name="settings" />
            <span>YAML</span>
          </div>
          <CodeBlock
            lang="yaml"
            content="robot:
  nombre: TurtleBot
  velocidad: 2.5
  sensores:
    - lidar
    - camara
  bateria:
    voltaje: 12.4
    nivel: 85"
          />
        </div>

        <div class="format-example json">
          <div class="format-label">
            <q-icon name="code" />
            <span>JSON</span>
          </div>
          <CodeBlock
            lang="json"
            content='{
  "robot": {
    "nombre": "TurtleBot",
    "velocidad": 2.5,
    "sensores": [
      "lidar",
      "camara"
    ],
    "bateria": {
      "voltaje": 12.4,
      "nivel": 85
    }
  }
}'
          />
        </div>

        <div class="format-example xml">
          <div class="format-label">
            <q-icon name="description" />
            <span>XML</span>
          </div>
          <CodeBlock
            lang="xml"
            content="<robot>
  <nombre>TurtleBot</nombre>
  <velocidad>2.5</velocidad>
  <sensores>
    <sensor>lidar</sensor>
    <sensor>camara</sensor>
  </sensores>
  <bateria>
    <voltaje>12.4</voltaje>
    <nivel>85</nivel>
  </bateria>
</robot>"
          />
        </div>
      </div>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>8. Errores Comunes</SectionTitle>

      <div class="errors-grid">
        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="red-4" size="lg" />
            <span>Perder tipos de datos</span>
          </div>
          <div class="error-content">
            <strong>Problema:</strong> YAML interpreta <code>true</code> como booleano, JSON lo
            mantiene, pero XML lo convierte a string. <br /><br />
            <strong>Solución:</strong> Validar tipos después de convertir
            <CodeBlock
              lang="python"
              content="# Verificar tipo
if isinstance(data['activo'], bool):
    print('Es booleano')
else:
    # Convertir si es string
    data['activo'] = data['activo'].lower() == 'true'"
            />
          </div>
        </div>

        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="orange-4" size="lg" />
            <span>Encoding issues</span>
          </div>
          <div class="error-content">
            <strong>Problema:</strong> Caracteres especiales (ñ, á, 中文) se rompen al convertir.
            <br /><br />
            <strong>Solución:</strong> Especificar encoding UTF-8
            <CodeBlock
              lang="python"
              content="# Siempre usar encoding
with open('data.yaml', 'r', encoding='utf-8') as f:
    data = yaml.safe_load(f)

with open('data.json', 'w', encoding='utf-8') as f:
    json.dump(data, f, ensure_ascii=False, indent=2)"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Conversión de Formatos en ROS 2"
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
          <code>yaml.safe_load()</code>
          <span>YAML → Dict</span>
        </div>
        <div class="summary-item">
          <code>yaml.dump()</code>
          <span>Dict → YAML</span>
        </div>
        <div class="summary-item">
          <code>json.load()</code>
          <span>JSON → Dict</span>
        </div>
        <div class="summary-item">
          <code>json.dump()</code>
          <span>Dict → JSON</span>
        </div>
        <div class="summary-item">
          <code>jq</code>
          <span>Procesar JSON en terminal</span>
        </div>
        <div class="summary-item">
          <code>yq</code>
          <span>Convertir YAML ↔ JSON</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Conversión" class="q-mt-lg">
        ✅ Usar <code>encoding='utf-8'</code> siempre
        <br />
        ✅ Validar tipos de datos después de convertir
        <br />
        ✅ <code>yaml.safe_load()</code> (no <code>yaml.load()</code> por seguridad)
        <br />
        ✅ <code>indent=2</code> para JSON legible
        <br />
        ✅ <code>default_flow_style=False</code> para YAML legible
        <br />
        ✅ Probar con <code>jq</code> o <code>yq</code> antes de código
      </AlertBlock>
    </div>
    <!-- ========== CTA FINAL ========== -->
    <div class="section-group q-mt-xl self-stretch column items-center">
      <div class="final-cta">
        <q-icon name="celebration" size="xl" color="primary" class="q-mb-md" />
        <h2 class="text-h4 text-white text-center q-mb-md">¡Has finalizado el módulo! 🎉</h2>
        <p class="text-body1 text-grey-4 text-center q-mb-lg">
          Has finalizado el módulo de Formato de datos.
        </p>

        <div class="row q-gutter-md justify-center">
          <q-btn
            color="primary"
            unelevated
            rounded
            size="lg"
            padding="14px 40px"
            to="/modulo-3/01conceptosPage"
            icon="rocket_launch"
            label="Comenzar con Git y GitHub"
            class="text-weight-bold"
          />
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

/* HUB DIAGRAM */
.hub-diagram {
  position: relative;
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 4rem 2rem;
  margin-top: 1.5rem;
  min-height: 300px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.hub-center {
  position: relative;
  z-index: 10;
}

.hub-circle {
  width: 120px;
  height: 120px;
  background: linear-gradient(135deg, #a855f7, #7c3aed);
  border-radius: 50%;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  box-shadow: 0 0 40px rgba(168, 85, 247, 0.5);
  border: 4px solid rgba(30, 41, 59, 0.8);
}

.hub-label {
  color: var(--text-primary);
  font-weight: 700;
  font-size: 0.9rem;
  margin-top: 0.5rem;
}

.hub-formats {
  position: absolute;
  width: 100%;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: space-around;
  padding: 0 2rem;
}

.format-node {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  padding: 1rem;
  background: var(--bg-surface);
  border: 2px solid rgba(148, 163, 184, 0.3);
  border-radius: 12px;
  color: var(--text-primary);
  font-weight: 700;
  z-index: 5;
}

.format-node.yaml {
  border-color: #3b82f6;
  color: var(--text-info, #2563eb);
}

.format-node.json {
  border-color: var(--text-warning, #d97706);
  color: var(--text-warning, #d97706);
}

.format-node.xml {
  border-color: #ef4444;
  color: #f87171;
}

/* CONVERSION GRID */
.conversion-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.conversion-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  display: flex;
  flex-direction: column;
}

.conversion-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: var(--text-primary);
}

.conversion-header.yaml {
  background: rgba(59, 130, 246, 0.2);
  border-bottom-color: #3b82f6;
}

.conversion-header.json {
  background: rgba(251, 191, 36, 0.2);
  border-bottom-color: var(--text-warning, #d97706);
}

/* EXAMPLE CARDS */
.example-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
  display: flex;
  flex-direction: column;
}

.example-card.input {
  border-top: 4px solid #3b82f6;
}

.example-card.output {
  border-top: 4px solid #fbbf24;
}

.example-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: var(--text-primary);
}

/* TOOLS GRID */
.tools-grid {
  display: flex;
  flex-direction: column;
  gap: 2rem;
  margin-top: 1.5rem;
}

.tool-item {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 2rem;
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 2rem;
  align-items: start;
}

.tool-item.jq {
  border-left: 4px solid #22c55e;
}

.tool-item.yq {
  border-left: 4px solid #3b82f6;
}

.tool-badge {
  font-family: 'Fira Code', monospace;
  font-size: 2rem;
  font-weight: 700;
  padding: 1rem 2rem;
  background: var(--bg-surface-hover);
  border-radius: 8px;
  border: 2px solid;
}

.tool-item.jq .tool-badge {
  color: var(--text-code);
  border-color: var(--text-code);
}

.tool-item.yq .tool-badge {
  color: #3b82f6;
  border-color: #3b82f6;
}

.tool-title {
  font-size: 1.3rem;
  font-weight: 700;
  color: var(--text-primary);
  margin-bottom: 0.5rem;
}

.tool-desc {
  color: var(--text-secondary);
  margin-bottom: 1rem;
}

/* FORMAT COMPARISON */
.format-comparison {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.format-example {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.format-example.yaml {
  border-top: 4px solid #3b82f6;
}

.format-example.json {
  border-top: 4px solid #fbbf24;
}

.format-example.xml {
  border-top: 4px solid #ef4444;
}

.format-label {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: var(--text-primary);
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

.error-content {
  padding: 1.5rem;
  color: var(--text-secondary);
}

.error-content strong {
  color: var(--text-primary);
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

/* COMPARISON TABLE */
.comparison-table {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.comparison-row {
  display: grid;
  grid-template-columns: 1.5fr 1fr 1fr 1fr;
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
  font-size: 0.85rem;
}

@media (max-width: 768px) {
  .hub-formats {
    position: static;
    flex-direction: column;
    gap: 1rem;
    margin-top: 2rem;
  }

  .tool-item {
    grid-template-columns: 1fr;
    gap: 1rem;
  }

  .comparison-row {
    grid-template-columns: 1fr;
  }

  .comparison-cell {
    border-right: none;
    border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  }
}
/* ========== CTA FINAL ========== */
.final-cta {
  text-align: center;
  margin: 0 auto;
}
</style>
