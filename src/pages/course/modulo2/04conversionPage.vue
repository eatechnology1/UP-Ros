<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        En un proyecto ROS 2 real usarás <strong>los tres formatos simultáneamente</strong>:
        YAML para parámetros de nodos, JSON para APIs y rosbridge, XML para URDF y launch files.
        Convertir entre ellos correctamente — sin perder tipos ni datos — es una habilidad crítica
        para depurar e integrar sistemas.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div v-for="f in facts" :key="f.label" class="fact-pill">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         01 HUB CENTRAL
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        Python Dict — El Formato Universal de Conversión
      </SectionTitle>

      <TextBlock>
        No se convierte directamente YAML → JSON. Siempre se carga primero a un
        <strong>diccionario Python</strong>, se manipulan los datos si hace falta,
        y luego se exporta al formato destino. El dict es el pivote.
      </TextBlock>

      <!-- Hub visual -->
      <div class="hub-wrap q-mt-lg">
        <div class="hub-top-row">
          <div v-for="fmt in hubFormats" :key="fmt.name" class="htr-slot">
            <div class="hub-node" :style="{ '--hn-color': fmt.color }">
              <q-icon :name="fmt.icon" size="22px" :style="{ color: fmt.color }" />
              <span class="hn-name">{{ fmt.name }}</span>
              <span class="hn-use">{{ fmt.use }}</span>
            </div>
            <div v-if="fmt !== hubFormats[hubFormats.length - 1]" class="hub-arrow-h">
              <div class="hah-line"></div>
              <div class="hah-fns">
                <code v-for="fn in fmt.fns" :key="fn">{{ fn }}</code>
              </div>
              <div class="hah-line"></div>
            </div>
          </div>
        </div>
        <div class="hub-xml-row">
          <div class="hub-vert-conn">
            <div class="hvc-line"></div>
            <div class="hvc-fns">
              <code>ET.parse() → helper()</code>
              <code>dict → ET.tostring()</code>
            </div>
            <div class="hvc-line"></div>
          </div>
          <div class="hub-node hub-node-xml" :style="{ '--hn-color': '#f87171' }">
            <q-icon name="description" size="22px" style="color:#f87171" />
            <span class="hn-name">XML</span>
            <span class="hn-use">URDF · Launch</span>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         02 CONVERSIONES BÁSICAS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        Conversiones Básicas — Los 6 Caminos
      </SectionTitle>

      <TextBlock>
        Cada par de formatos tiene dos funciones: una para leer (→ Dict) y otra para escribir
        (Dict →). Dominar estos 6 caminos cubre el 95% de las conversiones en ROS 2:
      </TextBlock>

      <div class="conv-grid q-mt-lg">
        <div v-for="conv in conversions" :key="conv.from + conv.to" class="conv-card"
          :style="{ '--conv-color': conv.color }">
          <div class="cc-header">
            <code class="cc-from">{{ conv.from }}</code>
            <q-icon :name="conv.dir === '→' ? 'arrow_forward' : 'arrow_back'"
              size="18px" :style="{ color: conv.color }" />
            <code class="cc-to">{{ conv.to }}</code>
          </div>
          <div class="cc-fn" :style="{ color: conv.color }">
            <code>{{ conv.fn }}</code>
          </div>
          <CodeBlock :hide-header="true" lang="python" :content="conv.code" :copyable="true" />
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         03 PIPELINE ROS 2
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        Pipeline Completo — Parámetros ROS 2 → API JSON
      </SectionTitle>

      <TextBlock>
        Escenario real: tienes parámetros de Nav2 en YAML y quieres exponerlos en una API REST
        para que un dashboard web los visualice y modifique. Este script hace el pipeline completo:
      </TextBlock>

      <!-- Input / Output side by side -->
      <div class="io-grid q-mt-lg">
        <div class="io-card io-input">
          <div class="ioc-header">
            <q-icon name="settings" size="14px" color="primary" />
            Input: nav_params.yaml
          </div>
          <CodeBlock :hide-header="true" lang="yaml" :content="navYamlCode" />
        </div>
        <div class="io-arrow">
          <q-icon name="arrow_forward" size="24px" style="color:var(--text-muted)" />
          <div class="ioa-label">Python script</div>
        </div>
        <div class="io-card io-output">
          <div class="ioc-header">
            <q-icon name="code" size="14px" color="warning" />
            Output: nav2_controller.json
          </div>
          <CodeBlock :hide-header="true" lang="json" :content="navJsonCode" />
        </div>
      </div>

      <CodeBlock title="ros2_params_to_api.py — Pipeline completo"
        lang="python" :content="pipelineCode" :copyable="true" class="q-mt-xl" />
    </div>

    <!-- ══════════════════════════════════════════
         04 MISMO DATO — 3 FORMATOS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        El Mismo Robot en 3 Formatos
      </SectionTitle>

      <TextBlock>
        La misma información estructurada se ve radicalmente distinta en cada formato.
        Observa cómo cambia la verbosidad, la legibilidad y el tipo de los valores:
      </TextBlock>

      <div class="formats-grid q-mt-lg">
        <div v-for="fmt in threeFormats" :key="fmt.name" class="fmt-card"
          :style="{ '--fmt-color': fmt.color }">
          <div class="fmtc-header">
            <q-icon :name="fmt.icon" size="18px" :style="{ color: fmt.color }" />
            <span class="fmtc-name">{{ fmt.name }}</span>
            <span class="fmtc-chars" :style="{ background: fmt.color + '18', color: fmt.color }">{{ fmt.chars }}</span>
          </div>
          <CodeBlock :hide-header="true" :lang="fmt.lang" :content="fmt.code" />
          <div class="fmtc-note">
            <q-icon name="star" size="12px" :style="{ color: fmt.color }" class="q-mr-xs" />
            {{ fmt.note }}
          </div>
        </div>
      </div>

      <!-- Type preservation -->
      <div class="type-table q-mt-xl">
        <div class="tt-title">
          <q-icon name="warning_amber" size="16px" color="warning" />
          Preservación de tipos al convertir — pitfalls críticos
        </div>
        <div class="tt-body">
          <div class="tt-row tt-header">
            <div class="tt-cell">Valor original (YAML)</div>
            <div class="tt-cell">Python Dict</div>
            <div class="tt-cell">→ JSON</div>
            <div class="tt-cell">→ XML texto</div>
          </div>
          <div v-for="row in typeRows" :key="row.yaml" class="tt-row">
            <div class="tt-cell"><code>{{ row.yaml }}</code></div>
            <div class="tt-cell">
              <code :style="{ color: row.pyColor }">{{ row.python }}</code>
            </div>
            <div class="tt-cell">
              <code :style="{ color: row.jsonOk ? '#4ade80' : '#fbbf24' }">{{ row.json }}</code>
              <q-icon v-if="!row.jsonOk" name="warning" size="12px" color="warning" class="q-ml-xs" />
            </div>
            <div class="tt-cell">
              <code style="color:#f87171">{{ row.xml }}</code>
              <q-icon name="error" size="12px" color="negative" class="q-ml-xs" />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         05 CLI: jq + yq
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        CLI — Convertir sin Código Python
      </SectionTitle>

      <TextBlock>
        Para conversiones rápidas en la terminal — durante depuración o scripting bash —
        <code>jq</code> y <code>yq</code> son insustituibles. También puedes usar
        one-liners de Python sin crear un archivo:
      </TextBlock>

      <div class="cli-grid q-mt-lg">
        <div v-for="tool in cliTools" :key="tool.name" class="cli-card"
          :style="{ '--cli-color': tool.color }">
          <div class="clic-header">
            <code class="clic-badge">{{ tool.badge }}</code>
            <div>
              <div class="clic-name">{{ tool.name }}</div>
              <div class="clic-desc">{{ tool.desc }}</div>
            </div>
          </div>
          <CodeBlock :hide-header="true" lang="bash" :content="tool.code" :copyable="true" />
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         06 TABLA COMPARATIVA
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">06</span>
        Comparación Completa — Cuándo Usar Cada Formato
      </SectionTitle>

      <div class="comp-table q-mt-lg">
        <div class="cpt-row cpt-header">
          <div class="cpt-cell">Característica</div>
          <div class="cpt-cell cpt-yaml">YAML</div>
          <div class="cpt-cell cpt-json">JSON</div>
          <div class="cpt-cell cpt-xml">XML</div>
        </div>
        <div v-for="row in compRows" :key="row.feature" class="cpt-row">
          <div class="cpt-cell cpt-feat">{{ row.feature }}</div>
          <div v-for="val in [row.yaml, row.json, row.xml]" :key="val" class="cpt-cell">
            <span :class="val.startsWith('✅') ? 'cpt-yes' : val.startsWith('❌') ? 'cpt-no' : 'cpt-neutral'">
              {{ val }}
            </span>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Diccionario de Errores de Conversión</SectionTitle>

      <div class="error-list q-mt-lg">
        <div v-for="(err, i) in commonErrors" :key="i" class="error-item"
          :style="{ '--err-color': err.color }">
          <div class="err-header" @click="err.open = !err.open">
            <div class="err-left">
              <div class="err-num" :style="{ background: err.color + '18', color: err.color }">
                {{ i + 1 }}
              </div>
              <div>
                <div class="err-type" :style="{ color: err.color }">{{ err.type }}</div>
                <div class="err-summary">{{ err.summary }}</div>
              </div>
            </div>
            <q-icon :name="err.open ? 'expand_less' : 'expand_more'"
              size="20px" style="color:var(--text-muted); flex-shrink:0" />
          </div>
          <div v-show="err.open" class="err-body">
            <div class="err-cause">
              <q-icon name="search" size="14px" class="q-mr-xs" />
              <strong>Causa:</strong> {{ err.cause }}
            </div>
            <CodeBlock :hide-header="true" lang="python" :content="err.code" />
            <div class="err-fix">
              <q-icon name="build" size="14px" class="q-mr-xs" color="positive" />
              <strong>Solución:</strong> {{ err.fix }}
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RETO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — Convertidor YAML ↔ JSON para Dashboard</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Crea un convertidor bidireccional de parámetros ROS 2</div>
            <div class="challenge-subtitle">
              YAML → JSON para API + JSON → YAML para cargar de vuelta al nodo
            </div>
          </div>
          <div class="challenge-badge">45 min</div>
        </div>

        <div class="challenge-steps q-mt-md">
          <div class="cs-title">Pasos:</div>
          <div class="cs-list">
            <div v-for="step in challengeSteps" :key="step.num" class="cs-item">
              <div class="cs-num" :style="{ background: step.color }">{{ step.num }}</div>
              <div class="cs-text">{{ step.text }}</div>
            </div>
          </div>
        </div>

        <CodeBlock title="Esqueleto para completar" lang="python"
          :content="challengeCode" :copyable="true" class="q-mt-md" />

        <q-expansion-item icon="lightbulb" label="Ver pistas"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div v-for="h in challengeHints" :key="h" class="answer-row">
              <q-icon name="chevron_right" size="14px" style="color:#4ade80" />
              {{ h }}
            </div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         VIDEO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Video Complementario</SectionTitle>
      <TextBlock>Conversión de formatos en ROS 2:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Conversión de formatos en ROS 2" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
            allowfullscreen></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" size="16px" color="info" class="q-mr-sm" />
          Video en progreso — será reemplazado con contenido del curso.
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RESUMEN
    ══════════════════════════════════════════ -->
    <div class="section-group q-mb-xl">
      <SectionTitle>Resumen — Funciones y Herramientas</SectionTitle>
      <div class="summary-grid q-mt-lg">
        <div v-for="s in summaryItems" :key="s.cmd" class="summary-card"
          :style="{ '--sc-color': s.color }">
          <code class="sc-cmd">{{ s.cmd }}</code>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-example">
            <q-icon name="arrow_right" size="14px" class="q-mr-xs" />{{ s.example }}
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         CTA FINAL
    ══════════════════════════════════════════ -->
    <div class="section-group q-mt-xl">
      <div class="final-cta">
        <div class="fca-icon">
          <q-icon name="celebration" size="48px" color="warning" />
        </div>
        <h2 class="fca-title">¡Módulo 2 completado!</h2>
        <p class="fca-sub">
          Dominas XML, JSON, YAML y la conversión entre ellos. Estás listo para
          comenzar con Git y GitHub — el siguiente pilar de todo proyecto de robótica profesional.
        </p>
        <div class="fca-actions">
          <q-btn color="primary" unelevated rounded size="lg" padding="14px 40px"
            to="/modulo-3/01conceptosPage"
            icon="rocket_launch" label="Comenzar Git y GitHub"
            class="text-weight-bold" />
        </div>
        <div class="fca-modules">
          <div v-for="m in completedModules" :key="m.name" class="fca-module"
            :style="{ '--fca-color': m.color }">
            <q-icon name="check_circle" size="16px" :style="{ color: m.color }" />
            {{ m.name }}
          </div>
        </div>
      </div>
    </div>

  </LessonContainer>
</template>

<script setup lang="ts">
import { reactive } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';

// ═══════════════════════════════════════════════════════════════
// CODE CONSTANTS (must be defined before data arrays that reference them)
// ═══════════════════════════════════════════════════════════════

const yamlToDictCode = [
  'import yaml',
  '',
  'with open("config.yaml", "r", encoding="utf-8") as f:',
  '    data = yaml.safe_load(f)',
  '',
  '# data es ahora un dict Python',
  'print(type(data))          # <class "dict">',
  'print(data["robot"]["vel"]) # 2.5  (float)',
].join('\n');

const dictToYamlCode = [
  'import yaml',
  '',
  'data = {',
  '    "robot": {"vel": 2.5, "activo": True},',
  '    "sensores": ["lidar", "camara"]',
  '}',
  '',
  'with open("output.yaml", "w", encoding="utf-8") as f:',
  '    yaml.dump(data, f,',
  '              default_flow_style=False,',
  '              allow_unicode=True)',
].join('\n');

const jsonToDictCode = [
  'import json',
  '',
  'with open("data.json", "r", encoding="utf-8") as f:',
  '    data = json.load(f)',
  '',
  '# data es un dict Python',
  'print(data["nombre"])   # "TurtleBot"',
  'print(data["activo"])   # True (bool)',
].join('\n');

const dictToJsonCode = [
  'import json',
  '',
  'data = {"robot": "TurtleBot", "activo": True, "vel": 2.5}',
  '',
  'with open("output.json", "w", encoding="utf-8") as f:',
  '    json.dump(data, f,',
  '              indent=2,',
  '              ensure_ascii=False)',
  '# ensure_ascii=False preserva ñ, á, etc.',
].join('\n');

const xmlToDictCode = [
  'import xml.etree.ElementTree as ET',
  '',
  'tree = ET.parse("robot.xml")',
  'root = tree.getroot()',
  '',
  '# Leer elementos manualmente',
  'data = {',
  '    "nombre": root.find("nombre").text,',
  '    "velocidad": float(root.find("velocidad").text),',
  '    # ⚠️ XML guarda todo como string',
  '    # → necesitas convertir tipos manualmente',
  '}',
].join('\n');

const dictToXmlCode = [
  'import xml.etree.ElementTree as ET',
  '',
  'data = {"nombre": "TurtleBot", "velocidad": 2.5}',
  '',
  'root = ET.Element("robot")',
  'for key, val in data.items():',
  '    child = ET.SubElement(root, key)',
  '    child.text = str(val)  # Todo a string',
  '',
  'tree = ET.ElementTree(root)',
  'ET.indent(tree, space="  ")  # Python 3.9+',
  'tree.write("robot.xml", encoding="unicode")',
].join('\n');

const navYamlCode = [
  'nav2_controller:',
  '  ros__parameters:',
  '    max_vel_x: 0.26',
  '    min_vel_x: -0.26',
  '    max_vel_theta: 1.0',
  '    xy_goal_tolerance: 0.25',
  '    yaw_goal_tolerance: 0.25',
  '    use_sim_time: false',
].join('\n');

const navJsonCode = [
  '{',
  '  "node": "nav2_controller",',
  '  "parameters": {',
  '    "max_vel_x": 0.26,',
  '    "min_vel_x": -0.26,',
  '    "max_vel_theta": 1.0,',
  '    "xy_goal_tolerance": 0.25,',
  '    "yaw_goal_tolerance": 0.25,',
  '    "use_sim_time": false',
  '  },',
  '  "converted_at": "2025-01-15T10:30:00"',
  '}',
].join('\n');

const pipelineCode = [
  '#!/usr/bin/env python3',
  '"""',
  'Pipeline: Parámetros ROS 2 (YAML) → API JSON para dashboard web.',
  'Uso: python3 ros2_to_api.py nav_params.yaml',
  '"""',
  'import yaml, json, sys, os',
  'from datetime import datetime',
  '',
  '',
  'def ros2_yaml_to_api_json(yaml_path: str) -> dict:',
  '    """Convierte params ROS 2 a estructura JSON para API REST."""',
  '    with open(yaml_path, "r", encoding="utf-8") as f:',
  '        raw = yaml.safe_load(f)',
  '',
  '    api = {',
  '        "source_file": os.path.basename(yaml_path),',
  '        "converted_at": datetime.now().isoformat(),',
  '        "ros_version": "Jazzy",',
  '        "nodes": {}',
  '    }',
  '',
  '    for node_name, config in raw.items():',
  '        if not isinstance(config, dict):',
  '            continue',
  '        params = config.get("ros__parameters", {})',
  '        if not params:',
  '            print(f"⚠️ {node_name}: sin ros__parameters", file=sys.stderr)',
  '            continue',
  '',
  '        api["nodes"][node_name] = {',
  '            "parameters": params,',
  '            "param_count": len(params)',
  '        }',
  '',
  '    return api',
  '',
  '',
  'def api_json_to_ros2_yaml(json_path: str) -> dict:',
  '    """Convierte la respuesta JSON de vuelta a YAML de ROS 2."""',
  '    with open(json_path, "r", encoding="utf-8") as f:',
  '        api = json.load(f)',
  '',
  '    ros2_params = {}',
  '    for node_name, node_data in api["nodes"].items():',
  '        ros2_params[node_name] = {',
  '            "ros__parameters": node_data["parameters"]',
  '        }',
  '    return ros2_params',
  '',
  '',
  'if __name__ == "__main__":',
  '    if len(sys.argv) < 2:',
  '        print("Uso: python3 ros2_to_api.py <params.yaml>")',
  '        sys.exit(1)',
  '',
  '    yaml_file = sys.argv[1]',
  '    api_data = ros2_yaml_to_api_json(yaml_file)',
  '',
  '    # Exportar JSON',
  '    json_out = yaml_file.replace(".yaml", "_api.json")',
  '    with open(json_out, "w", encoding="utf-8") as f:',
  '        json.dump(api_data, f, indent=2, ensure_ascii=False)',
  '',
  '    print(f"✅ Exportado: {json_out}")',
  '    print(f"   Nodos: {list(api_data[\'nodes\'].keys())}")',
].join('\n');

const yamlDataCode = [
  '# Más legible para humanos',
  'robot:',
  '  nombre: TurtleBot',
  '  velocidad: 2.5       # float',
  '  activo: true         # bool',
  '  sensores:',
  '    - lidar',
  '    - camara',
  '  bateria:',
  '    voltaje: 12.4',
  '    nivel: 85',
].join('\n');

const jsonDataCode = [
  '{',
  '  "robot": {',
  '    "nombre": "TurtleBot",',
  '    "velocidad": 2.5,',
  '    "activo": true,',
  '    "sensores": ["lidar", "camara"],',
  '    "bateria": {',
  '      "voltaje": 12.4,',
  '      "nivel": 85',
  '    }',
  '  }',
  '}',
].join('\n');

const xmlDataCode = [
  '<robot>',
  '  <nombre>TurtleBot</nombre>',
  '  <!-- Todo es string en XML -->',
  '  <velocidad>2.5</velocidad>',
  '  <activo>true</activo>',
  '  <sensores>',
  '    <sensor>lidar</sensor>',
  '    <sensor>camara</sensor>',
  '  </sensores>',
  '  <bateria>',
  '    <voltaje>12.4</voltaje>',
  '    <nivel>85</nivel>',
  '  </bateria>',
  '</robot>',
].join('\n');

const challengeCode = [
  '#!/usr/bin/env python3',
  '"""',
  'RETO: Convertidor bidireccional YAML ↔ JSON para parámetros ROS 2.',
  '"""',
  'import yaml, json, sys',
  '',
  '',
  'def yaml_to_json(yaml_path: str, json_path: str) -> None:',
  '    """TODO 1: Lee el YAML con yaml.safe_load() y encoding UTF-8."""',
  '    # with open(yaml_path, ...) as f:',
  '    #     data = ...',
  '',
  '    """TODO 2: Agrega metadatos: converted_at (datetime), source_file."""',
  '    # data["_meta"] = {...}',
  '',
  '    """TODO 3: Escribe JSON con indent=2 y ensure_ascii=False."""',
  '    # with open(json_path, ...) as f:',
  '    #     json.dump(...)',
  '',
  '',
  'def json_to_yaml(json_path: str, yaml_path: str) -> None:',
  '    """TODO 4: Lee el JSON con json.load()."""',
  '    # with open(json_path, ...) as f:',
  '    #     data = ...',
  '',
  '    """TODO 5: Elimina el campo _meta antes de escribir el YAML."""',
  '    # data.pop("_meta", None)',
  '',
  '    """TODO 6: Escribe YAML con default_flow_style=False y allow_unicode=True."""',
  '    # with open(yaml_path, ...) as f:',
  '    #     yaml.dump(...)',
  '',
  '',
  'if __name__ == "__main__":',
  '    # TODO 7: Parsea argumentos: python3 converter.py yaml2json in.yaml out.json',
  '    #         o python3 converter.py json2yaml in.json out.yaml',
  '    pass',
].join('\n');

// ═══════════════════════════════════════════════════════════════
// DATA ARRAYS
// ═══════════════════════════════════════════════════════════════

const facts = [
  { icon: '🔄', label: 'Python Dict — pivote universal: todo formato pasa por él' },
  { icon: '🛠️', label: 'yq + jq — convierte entre formatos directamente en la terminal' },
  { icon: '⚡', label: 'ros2 param dump — exporta parámetros en vivo a YAML automáticamente' },
];

const hubFormats = [
  { name: 'YAML', icon: 'settings', color: '#60a5fa', use: 'Parámetros · Launch', fns: ['yaml.safe_load()', 'yaml.dump()'] },
  { name: 'Python Dict', icon: 'data_object', color: '#c084fc', use: 'Formato intermedio universal', fns: ['json.load()', 'json.dump()'] },
  { name: 'JSON', icon: 'code', color: '#fbbf24', use: 'rosbridge · APIs · bags', fns: [] },
];

const conversions = [
  { from: 'YAML', to: 'Python Dict',  color: '#60a5fa', dir: '→', fn: 'yaml.safe_load(f)',        code: yamlToDictCode },
  { from: 'Python Dict', to: 'YAML',  color: '#60a5fa', dir: '←', fn: 'yaml.dump(data, f)',       code: dictToYamlCode },
  { from: 'JSON', to: 'Python Dict',  color: '#fbbf24', dir: '→', fn: 'json.load(f)',             code: jsonToDictCode },
  { from: 'Python Dict', to: 'JSON',  color: '#fbbf24', dir: '←', fn: 'json.dump(data, f)',       code: dictToJsonCode },
  { from: 'XML',  to: 'Python Dict',  color: '#f87171', dir: '→', fn: 'ET.parse() → helper()',   code: xmlToDictCode },
  { from: 'Python Dict', to: 'XML',   color: '#f87171', dir: '←', fn: 'ET.SubElement() + write()', code: dictToXmlCode },
];

const threeFormats = [
  { name: 'YAML', icon: 'settings',   color: '#60a5fa', lang: 'yaml', chars: '165 chars',
    note: 'Más legible — comentarios, tipos nativos, sin comillas en strings',
    code: yamlDataCode },
  { name: 'JSON', icon: 'code',        color: '#fbbf24', lang: 'json', chars: '198 chars',
    note: 'Más explícito — tipos correctos (bool/float), ideal para APIs',
    code: jsonDataCode },
  { name: 'XML',  icon: 'description', color: '#f87171', lang: 'xml',  chars: '312 chars',
    note: 'Más verboso — todo es string, necesita conversión manual de tipos',
    code: xmlDataCode },
];

const typeRows = [
  { yaml: 'activo: true',  python: 'True (bool)',    pyColor: '#c084fc', json: 'true ✅',      jsonOk: true,  xml: '"true" (str) ❌' },
  { yaml: 'vel: 2.5',      python: '2.5 (float)',    pyColor: '#60a5fa', json: '2.5 ✅',       jsonOk: true,  xml: '"2.5" (str) ❌' },
  { yaml: 'nivel: 85',     python: '85 (int)',        pyColor: '#60a5fa', json: '85 ✅',        jsonOk: true,  xml: '"85" (str) ❌' },
  { yaml: 'activo: yes',   python: 'True (bool)',    pyColor: '#fbbf24', json: 'true ⚠️',     jsonOk: false, xml: '"True" (str) ❌' },
  { yaml: 'error: ~',      python: 'None',            pyColor: '#94a3b8', json: 'null ✅',      jsonOk: true,  xml: '""  (str) ❌' },
];

const cliTools = [
  {
    name: 'jq — Procesar JSON', badge: 'jq', color: '#4ade80', desc: 'Filtrar, transformar y validar JSON desde terminal',
    code: [
      '# Instalar',
      'sudo apt install jq',
      '',
      '# Pretty-print y validar',
      'jq . config.json',
      '',
      '# Extraer campo anidado',
      "jq '.robot.velocidad' config.json",
      '',
      '# Convertir parámetros de nodo ROS 2 exportado',
      "jq '.nav2_controller.ros__parameters' params.json",
    ].join('\n'),
  },
  {
    name: 'yq — Procesar YAML', badge: 'yq', color: '#60a5fa', desc: 'Como jq pero para YAML, con conversión de formatos',
    code: [
      '# Instalar',
      'pip install yq',
      '',
      '# YAML → JSON',
      'yq -o=json robot_params.yaml > robot_params.json',
      '',
      '# JSON → YAML',
      'yq -P data.json > data.yaml',
      '',
      '# Extraer campo YAML',
      "yq '.nav2_controller.ros__parameters.max_vel_x' params.yaml",
    ].join('\n'),
  },
  {
    name: 'Python one-liners', badge: 'py', color: '#c084fc', desc: 'Sin crear archivos — directo en la terminal',
    code: [
      '# YAML → JSON one-liner',
      'python3 -c "import yaml,json,sys; \\',
      '  json.dump(yaml.safe_load(sys.stdin),sys.stdout,indent=2)" \\',
      '  < config.yaml',
      '',
      '# ros2 param dump → validar con jq',
      'ros2 param dump /mi_nodo --stdout | \\',
      '  python3 -c "import yaml,json,sys; \\',
      '  json.dump(yaml.safe_load(sys.stdin),sys.stdout,indent=2)" | \\',
      '  jq .',
    ].join('\n'),
  },
];

const compRows = [
  { feature: 'Legibilidad humana',  yaml: '⭐⭐⭐⭐⭐ máxima',   json: '⭐⭐⭐ media',         xml: '⭐⭐ mínima' },
  { feature: 'Comentarios',         yaml: '✅ con #',             json: '❌ no soportado',     xml: '✅ con <!-- -->' },
  { feature: 'Tipos nativos',       yaml: '✅ bool/int/float/str', json: '✅ bool/int/float/str', xml: '❌ todo string' },
  { feature: 'Strings multilínea',  yaml: '✅ | y >',             json: '❌ solo \\n escapado', xml: '✅ CDATA' },
  { feature: 'Anclas/DRY',          yaml: '✅ & y *',             json: '❌ repetir',          xml: '❌ repetir' },
  { feature: 'Esquemas',            yaml: '❌ ninguno nativo',    json: '✅ JSON Schema',      xml: '✅ XSD / DTD' },
  { feature: 'Verbosidad',          yaml: 'Mínima',               json: 'Media',               xml: 'Máxima' },
  { feature: 'Uso principal ROS 2', yaml: 'Parámetros · Launch',  json: 'rosbridge · APIs',   xml: 'URDF · Launch XML' },
  { feature: 'Parseo Web/JS',       yaml: '❌ necesita librería', json: '✅ nativo en JS',     xml: '✅ DOMParser' },
];

const commonErrors = reactive([
  {
    type: 'Pérdida de tipos en XML',
    summary: 'XML convierte todos los valores a strings — true se vuelve "true" (texto)',
    color: '#f87171',
    cause: 'XML no tiene concepto de tipos de dato. Todo es texto. Al parsear con ElementTree, .text siempre devuelve str.',
    code: [
      '# ❌ Sin conversión — activo es string "true" no bool',
      'root = ET.parse("robot.xml").getroot()',
      'activo = root.find("activo").text  # "true" (str!)',
      'if activo:  # ¡Siempre True porque string no vacío!',
      '    ...',
      '',
      '# ✅ Con conversión explícita de tipos',
      'activo_str = root.find("activo").text',
      'activo = activo_str.lower() == "true"  # bool Python',
      'velocidad = float(root.find("velocidad").text)  # float',
    ].join('\n'),
    fix: 'Siempre convierte manualmente los valores de XML: float(), int(), o == "true" para booleanos.',
    open: false,
  },
  {
    type: 'Encoding — caracteres especiales',
    summary: 'Caracteres como ñ, á, 中文 se rompen si no se especifica UTF-8 explícitamente',
    color: '#fbbf24',
    cause: 'Python 3 abre archivos con el encoding del sistema (que puede ser ASCII en algunos servers). En Linux español, normalmente es UTF-8, pero en sistemas cloud puede no serlo.',
    code: [
      '# ❌ Sin encoding — puede romper en algunos sistemas',
      'with open("config.yaml", "r") as f:',
      '    data = yaml.safe_load(f)',
      '',
      '# ✅ Siempre especificar encoding explícito',
      'with open("config.yaml", "r", encoding="utf-8") as f:',
      '    data = yaml.safe_load(f)',
      '',
      '# ✅ Y al escribir JSON con caracteres especiales',
      'json.dump(data, f, ensure_ascii=False, indent=2)',
      '# ensure_ascii=False preserva ñ, á en el JSON',
    ].join('\n'),
    fix: 'Añade encoding="utf-8" a todos los open() y ensure_ascii=False a json.dump(). Nunca asumas el encoding del sistema.',
    open: false,
  },
  {
    type: 'YAML boolean ambiguo — yes/no → True/False',
    summary: 'PyYAML parsea "yes", "no", "on", "off" como booleanos — no como strings',
    color: '#c084fc',
    cause: 'YAML 1.1 (usado por PyYAML) considera yes/no/on/off como True/False. Al convertir a JSON, salen como true/false en lugar de "yes"/"no". Rompe APIs que esperan strings específicos.',
    code: [
      '# YAML: confirmation: yes',
      'data = yaml.safe_load("confirmation: yes")',
      'print(data["confirmation"])  # True (bool!) no "yes"',
      '',
      '# Al convertir a JSON:',
      'print(json.dumps(data))  # {"confirmation": true}',
      '# La API esperaba "yes" — recibe true → ERROR',
      '',
      '# ✅ Solución: usar comillas en el YAML',
      '# confirmation: "yes"',
      '# O usar true/false explícitamente para booleanos',
    ].join('\n'),
    fix: 'En configs ROS 2, usa true/false para booleanos. Si el valor debe ser el string "yes"/"no", ponlo entre comillas en el YAML.',
    open: false,
  },
  {
    type: 'Claves duplicadas en YAML',
    summary: 'YAML permite claves duplicadas — Python silenciosamente toma la última y descarta las anteriores',
    color: '#f97316',
    cause: 'El estándar YAML dice que las claves duplicadas son "undefined behavior". PyYAML toma la última. JSON las rechaza. Si copias-pegas bloques, es fácil duplicar sin darte cuenta.',
    code: [
      '# YAML con clave duplicada (yamllint detecta esto)',
      '# robot:',
      '#   velocidad: 2.0',
      '#   velocidad: 3.0  ← clave duplicada!',
      '',
      'data = yaml.safe_load("""',
      'robot:',
      '  velocidad: 2.0',
      '  velocidad: 3.0',
      '""")',
      'print(data["robot"]["velocidad"])  # 3.0 (no 2.0!)',
      '# La primera definición se PIERDE silenciosamente',
      '',
      '# ✅ Detectar con yamllint:',
      '# yamllint -d "{extends: default}" params.yaml',
    ].join('\n'),
    fix: 'Ejecuta yamllint en tus archivos YAML. Detecta claves duplicadas y muchos otros problemas antes de que causen bugs en producción.',
    open: false,
  },
  {
    type: 'Profundidad excesiva — JSON y YAML difieren',
    summary: 'YAML colapsa objetos vacíos de forma distinta a JSON — None vs {} vs ""',
    color: '#4ade80',
    cause: 'Una clave YAML con solo un : y nada después se convierte en None (Python) → null (JSON). Una clave seguida de {} se convierte en {} (dict vacío). Esta diferencia sutil rompe código que espera un dict pero recibe None.',
    code: [
      '# YAML:',
      '# config:      ← None en Python',
      '# config: {}   ← {} en Python',
      '',
      'data = yaml.safe_load("config:")',
      'print(data["config"])  # None',
      '',
      'data2 = yaml.safe_load("config: {}")',
      'print(data2["config"])  # {}',
      '',
      '# ❌ El código asume dict pero recibe None',
      '# for k, v in data["config"].items():  # AttributeError!',
      '',
      '# ✅ Siempre verificar el tipo',
      'cfg = data.get("config") or {}',
    ].join('\n'),
    fix: 'Usa data.get("key") or {} para acceder a sub-objetos YAML que podrían ser None. Siempre valida el tipo antes de iterar.',
    open: false,
  },
]);

const challengeSteps = [
  { num: 1, color: '#60a5fa', text: 'Crea robot_params.yaml con 2+ nodos y sus ros__parameters' },
  { num: 2, color: '#fbbf24', text: 'Implementa yaml_to_json(): lee, agrega metadatos (_meta), escribe JSON con UTF-8' },
  { num: 3, color: '#4ade80', text: 'Implementa json_to_yaml(): elimina _meta, escribe YAML legible con default_flow_style=False' },
  { num: 4, color: '#c084fc', text: 'Parsea argumentos: python3 converter.py yaml2json in.yaml out.json' },
  { num: 5, color: '#f97316', text: 'Verifica con jq . output.json y yamllint output.yaml — sin errores' },
  { num: 6, color: '#f87171', text: '(Bonus) Carga el YAML regenerado con ros2 param load y verifica con ros2 param list' },
];

const challengeHints = [
  'Para agregar metadatos: data["_meta"] = {"converted_at": datetime.now().isoformat(), "version": "1.0"}',
  'Para eliminar metadatos: data.pop("_meta", None) — el None evita KeyError si no existe',
  'sys.argv[1] es el subcomando, sys.argv[2] el archivo de entrada, sys.argv[3] el de salida',
  'ensure_ascii=False en json.dump() preserva caracteres especiales (ñ, á, etc.)',
  'allow_unicode=True en yaml.dump() hace lo mismo para YAML',
  'yamllint --strict output.yaml valida que el YAML generado sea correcto',
];

const summaryItems = [
  { cmd: 'yaml.safe_load(f)',  desc: 'YAML → Python Dict (seguro)',          example: 'encoding="utf-8" siempre', color: '#60a5fa' },
  { cmd: 'yaml.dump(data, f)', desc: 'Python Dict → YAML legible',           example: 'default_flow_style=False', color: '#60a5fa' },
  { cmd: 'json.load(f)',        desc: 'JSON → Python Dict',                   example: 'tipos preservados',        color: '#fbbf24' },
  { cmd: 'json.dump(data, f)', desc: 'Python Dict → JSON',                   example: 'indent=2, ensure_ascii=False', color: '#fbbf24' },
  { cmd: 'ET.parse()',          desc: 'XML → árbol ElementTree',              example: '.text siempre es string',  color: '#f87171' },
  { cmd: 'ET.SubElement()',     desc: 'Construir XML desde dict',             example: 'str(val) para todo',       color: '#f87171' },
  { cmd: 'yq -o=json',         desc: 'YAML → JSON en terminal (sin Python)',  example: 'yq -o=json f.yaml',       color: '#4ade80' },
  { cmd: 'ros2 param dump',    desc: 'Exportar parámetros en vivo a YAML',   example: '--stdout para terminal',   color: '#c084fc' },
  { cmd: 'ensure_ascii=False', desc: 'Preservar ñ, á, 中文 en JSON',         example: 'json.dump(...)',            color: '#94a3b8' },
];

const completedModules = [
  { name: '2.1 XML — URDF y Launch',          color: '#f87171' },
  { name: '2.2 JSON — rosbridge y APIs',       color: '#fbbf24' },
  { name: '2.3 YAML — Parámetros ROS 2',      color: '#60a5fa' },
  { name: '2.4 Conversión — Pipeline Python', color: '#4ade80' },
];
</script>

<style scoped>
/* ══════════════════════════════════════════
   BASE
══════════════════════════════════════════ */
.section-group { margin-bottom: 3.5rem; }

code {
  background: var(--bg-code); color: var(--text-code);
  padding: 2px 7px; border-radius: 5px;
  font-family: 'Fira Code', monospace; font-size: .9em;
}
.cmd-badge {
  display: inline-flex; align-items: center; justify-content: center;
  width: 28px; height: 28px; border-radius: 8px;
  font-size: .75rem; font-weight: 800; margin-right: 8px; vertical-align: middle;
}
.cmd-badge.green  { background: rgba( 74,222,128,.15); color: #4ade80; }
.cmd-badge.amber  { background: rgba(251,191, 36,.15); color: #fbbf24; }
.cmd-badge.cyan   { background: rgba( 34,211,238,.15); color: #22d3ee; }
.cmd-badge.purple { background: rgba(192,132,252,.15); color: #c084fc; }
.cmd-badge.red    { background: rgba(248,113,113,.15); color: #f87171; }

.fact-pills { display: flex; gap: 10px; flex-wrap: wrap; }
.fact-pill {
  display: flex; align-items: center; gap: 8px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 999px; padding: 7px 16px; font-size: .84rem; color: var(--text-secondary);
  transition: transform .2s;
}
.fact-pill:hover { transform: translateY(-2px); }
.fact-icon { font-size: 1rem; }

/* ══════════════════════════════════════════
   HUB VISUAL
══════════════════════════════════════════ */
.hub-wrap {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; padding: 2rem 1.5rem;
  display: flex; flex-direction: column; align-items: center; gap: 0;
}
.hub-top-row { display: flex; align-items: center; justify-content: center; flex-wrap: wrap; gap: 0; width: 100%; }
.htr-slot    { display: flex; align-items: center; }
.hub-node {
  background: color-mix(in srgb, var(--hn-color) 8%, var(--bg-surface));
  border: 1px solid color-mix(in srgb, var(--hn-color) 30%, transparent);
  border-radius: 14px; padding: 1rem 1.5rem;
  display: flex; flex-direction: column; align-items: center; gap: 4px; text-align: center;
}
.hn-name { font-size: .95rem; font-weight: 800; color: var(--text-primary); }
.hn-use  { font-size: .72rem; color: var(--text-muted); }
.hub-arrow-h {
  display: flex; flex-direction: column; align-items: center;
  padding: 0 6px; gap: 4px; min-width: 80px;
}
.hah-line { height: 2px; background: var(--border-medium); width: 100%; border-radius: 1px; }
.hah-fns  { display: flex; flex-direction: column; align-items: center; gap: 2px; }
.hah-fns code { font-size: .65rem; background: none; padding: 0; color: var(--text-muted); white-space: nowrap; }
.hub-xml-row  { display: flex; flex-direction: column; align-items: center; gap: 0; margin-top: 0; }
.hub-vert-conn {
  display: flex; flex-direction: column; align-items: center; gap: 4px; padding: 6px 0;
}
.hvc-line { width: 2px; height: 16px; background: var(--border-medium); border-radius: 1px; }
.hvc-fns  { display: flex; flex-direction: column; align-items: center; gap: 2px; }
.hvc-fns code { font-size: .65rem; background: none; padding: 0; color: var(--text-muted); white-space: nowrap; }
.hub-node-xml { }

/* ══════════════════════════════════════════
   CONVERSIONS GRID
══════════════════════════════════════════ */
.conv-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
.conv-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--conv-color); border-radius: 12px;
  display: flex; flex-direction: column; overflow: hidden; min-width: 0;
}
.cc-header {
  display: flex; align-items: center; gap: 8px; padding: 10px 14px;
  background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle); flex-shrink: 0;
  flex-wrap: wrap;
}
.cc-from, .cc-to { font-size: .82rem; font-weight: 700; background: none; padding: 0; }
.cc-fn { padding: 6px 14px; font-size: .8rem; background: var(--bg-surface-hover); border-bottom: 1px solid var(--border-subtle); }
.cc-fn code { background: none; padding: 0; font-size: .8rem; }

/* ══════════════════════════════════════════
   IO GRID
══════════════════════════════════════════ */
.io-grid { display: grid; grid-template-columns: 1fr auto 1fr; gap: 12px; align-items: center; }
.io-card { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; min-width: 0; }
.io-input  { border-top: 3px solid #60a5fa; }
.io-output { border-top: 3px solid #fbbf24; }
.ioc-header { display: flex; align-items: center; gap: 8px; padding: 10px 14px; background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle); font-size: .84rem; font-weight: 600; color: var(--text-secondary); }
.io-arrow   { display: flex; flex-direction: column; align-items: center; gap: 6px; }
.ioa-label  { font-size: .72rem; color: var(--text-muted); text-align: center; }

/* ══════════════════════════════════════════
   3 FORMATS GRID
══════════════════════════════════════════ */
.formats-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; }
.fmt-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--fmt-color); border-radius: 12px;
  display: flex; flex-direction: column; overflow: hidden; min-width: 0;
}
.fmtc-header { display: flex; align-items: center; gap: 8px; padding: 10px 14px; background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle); flex-wrap: wrap; }
.fmtc-name   { font-size: .9rem; font-weight: 800; color: var(--text-primary); }
.fmtc-chars  { font-size: .7rem; font-weight: 700; padding: 2px 7px; border-radius: 999px; margin-left: auto; white-space: nowrap; }
.fmtc-note   { display: flex; align-items: flex-start; padding: 8px 14px; font-size: .77rem; color: var(--text-muted); line-height: 1.4; border-top: 1px solid var(--border-subtle); }

/* Type table */
.type-table { }
.tt-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.tt-body  { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; }
.tt-row   { display: grid; grid-template-columns: 1.2fr 1fr 1fr 1.3fr; border-bottom: 1px solid var(--border-subtle); }
.tt-row:last-child { border-bottom: none; }
.tt-header { background: var(--bg-surface-solid); font-size: .82rem; font-weight: 700; color: var(--text-primary); }
.tt-cell   { padding: 8px 12px; border-right: 1px solid var(--border-subtle); font-size: .8rem; color: var(--text-secondary); display: flex; align-items: center; }
.tt-cell:last-child { border-right: none; }
.tt-cell code { background: none; padding: 0; }

/* ══════════════════════════════════════════
   CLI TOOLS
══════════════════════════════════════════ */
.cli-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; }
.cli-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--cli-color); border-radius: 14px;
  display: flex; flex-direction: column; overflow: hidden; min-width: 0;
}
.clic-header { display: flex; align-items: flex-start; gap: 12px; padding: 12px 14px; background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle); }
.clic-badge  { font-family: 'Fira Code', monospace; font-size: 1.1rem; font-weight: 900; color: var(--cli-color); background: color-mix(in srgb, var(--cli-color) 10%, var(--bg-surface)); border-radius: 8px; padding: 4px 10px; flex-shrink: 0; }
.clic-name   { font-size: .88rem; font-weight: 700; color: var(--text-primary); }
.clic-desc   { font-size: .78rem; color: var(--text-muted); line-height: 1.3; margin-top: 2px; }

/* ══════════════════════════════════════════
   COMPARISON TABLE
══════════════════════════════════════════ */
.comp-table { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; }
.cpt-row    { display: grid; grid-template-columns: 1.5fr 1fr 1fr 1fr; border-bottom: 1px solid var(--border-subtle); }
.cpt-row:last-child { border-bottom: none; }
.cpt-header { background: var(--bg-surface-solid); font-size: .82rem; font-weight: 700; }
.cpt-cell   { padding: 9px 14px; border-right: 1px solid var(--border-subtle); font-size: .82rem; color: var(--text-secondary); display: flex; align-items: center; }
.cpt-cell:last-child { border-right: none; }
.cpt-feat   { font-weight: 600; color: var(--text-primary); }
.cpt-yaml   { color: #60a5fa; font-weight: 700; }
.cpt-json   { color: #fbbf24; font-weight: 700; }
.cpt-xml    { color: #f87171; font-weight: 700; }
.cpt-yes    { color: #4ade80; }
.cpt-no     { color: #f87171; }
.cpt-neutral{ color: var(--text-secondary); }

/* ══════════════════════════════════════════
   ERROR LIST
══════════════════════════════════════════ */
.error-list { display: flex; flex-direction: column; gap: 10px; }
.error-item { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid var(--err-color); border-radius: 12px; overflow: hidden; }
.err-header { display: flex; align-items: center; justify-content: space-between; padding: .9rem 1.25rem; cursor: pointer; gap: 12px; transition: background .2s; }
.err-header:hover { background: var(--bg-surface-hover); }
.err-left   { display: flex; align-items: flex-start; gap: 10px; min-width: 0; }
.err-num    { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .8rem; font-weight: 800; display: flex; align-items: center; justify-content: center; }
.err-type   { font-size: .82rem; font-weight: 700; color: var(--text-primary); margin-bottom: 2px; }
.err-summary{ font-size: .78rem; color: var(--text-muted); }
.err-body   { padding: .9rem 1.4rem 1.1rem; border-top: 1px solid var(--border-subtle); display: flex; flex-direction: column; gap: 10px; }
.err-cause  { font-size: .86rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }
.err-fix    { font-size: .85rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }

/* ══════════════════════════════════════════
   CHALLENGE BOX
══════════════════════════════════════════ */
.challenge-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 20px; padding: 1.75rem; border-top: 3px solid #f59e0b; }
.challenge-header { display: flex; align-items: flex-start; gap: 1rem; flex-wrap: wrap; }
.challenge-icon   { width: 52px; height: 52px; background: rgba(245,158,11,.15); border-radius: 14px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.challenge-title  { font-size: 1.05rem; font-weight: 700; color: var(--text-primary); margin-bottom: 4px; }
.challenge-subtitle { font-size: .9rem; color: var(--text-secondary); }
.challenge-badge  { margin-left: auto; font-size: .72rem; font-weight: 800; letter-spacing: .07em; padding: 4px 12px; border-radius: 999px; white-space: nowrap; background: rgba(96,165,250,.12); color: #60a5fa; border: 1px solid rgba(96,165,250,.3); }
.challenge-steps  { background: var(--bg-surface-hover); border-radius: 12px; padding: 1rem 1.25rem; }
.cs-title { font-size: .88rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 10px; }
.cs-list  { display: flex; flex-direction: column; gap: 8px; }
.cs-item  { display: flex; align-items: flex-start; gap: 10px; }
.cs-num   { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .82rem; font-weight: 800; color: #1e1e1e; display: flex; align-items: center; justify-content: center; }
.cs-text  { font-size: .87rem; color: var(--text-secondary); padding-top: 3px; }
:deep(.answer-header) { background: rgba(34,197,94,.08); border: 1px solid rgba(34,197,94,.25); border-radius: 10px; color: #22c55e; }
.answer-body { background: var(--bg-surface-hover); padding: 1.1rem 1.25rem; border-radius: 0 0 10px 10px; display: flex; flex-direction: column; gap: 8px; }
.answer-row  { display: flex; align-items: baseline; gap: 8px; font-size: .88rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   VIDEO
══════════════════════════════════════════ */
.video-card    { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; padding: 1.25rem; overflow: hidden; }
.video-wrapper { position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; border-radius: 10px; background: #000; }
.video-wrapper iframe { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }
.video-caption { display: flex; align-items: center; margin-top: 12px; font-size: .82rem; color: var(--text-muted); padding: 8px 12px; background: var(--bg-surface-hover); border-radius: 8px; }

/* ══════════════════════════════════════════
   SUMMARY
══════════════════════════════════════════ */
.summary-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; }
.summary-card { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 4px solid var(--sc-color); border-radius: 12px; padding: 1rem 1.25rem; transition: all .25s; }
.summary-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.sc-cmd     { display: block; font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; margin-bottom: 5px; word-break: break-all; }
.sc-desc    { font-size: .81rem; color: var(--text-secondary); margin-bottom: 6px; line-height: 1.4; }
.sc-example { display: flex; align-items: center; font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   CTA FINAL
══════════════════════════════════════════ */
.final-cta {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 24px; padding: 3rem 2rem;
  display: flex; flex-direction: column; align-items: center; gap: 1rem; text-align: center;
}
.fca-icon   { width: 80px; height: 80px; background: rgba(245,158,11,.1); border-radius: 24px; display: flex; align-items: center; justify-content: center; }
.fca-title  { font-size: 1.6rem; font-weight: 800; color: var(--text-primary); margin: 0; }
.fca-sub    { font-size: .95rem; color: var(--text-secondary); max-width: 520px; line-height: 1.6; margin: 0; }
.fca-actions{ margin-top: .5rem; }
.fca-modules{ display: flex; flex-wrap: wrap; gap: 8px; justify-content: center; margin-top: .75rem; }
.fca-module {
  display: flex; align-items: center; gap: 6px; font-size: .78rem; color: var(--text-muted);
  background: color-mix(in srgb, var(--fca-color) 8%, var(--bg-surface));
  border: 1px solid color-mix(in srgb, var(--fca-color) 20%, transparent);
  border-radius: 999px; padding: 4px 12px;
}

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1100px) {
  .conv-grid  { grid-template-columns: repeat(2, 1fr); }
  .cli-grid   { grid-template-columns: repeat(2, 1fr); }
}
@media (max-width: 900px) {
  .formats-grid { grid-template-columns: 1fr; }
  .summary-grid { grid-template-columns: repeat(2, 1fr); }
  .tt-row    { grid-template-columns: 1fr 1fr; }
  .hub-top-row { flex-direction: column; }
  .hah-line  { width: 2px; height: 16px; }
}
@media (max-width: 768px) {
  .conv-grid { grid-template-columns: 1fr; }
  .io-grid   { grid-template-columns: 1fr; }
  .io-arrow  { transform: rotate(90deg); }
  .cli-grid  { grid-template-columns: 1fr; }
  .cpt-row   { grid-template-columns: 1fr; }
  .cpt-cell  { border-right: none; border-bottom: 1px solid var(--border-subtle); }
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
}
@media (max-width: 480px) {
  .summary-grid { grid-template-columns: 1fr; }
  .tt-row    { grid-template-columns: 1fr; }
}
</style>
