<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-yellow-4 text-weight-bold q-mb-sm">
          MÓDULO 2.2: EL MENSAJERO ÁGIL
        </div>

        <h1 class="hero-title">JSON <span class="text-white">Básico</span></h1>

        <TextBlock>
          JSON (JavaScript Object Notation) es el estándar mundial para mover datos en internet. En
          Robótica, es el puente entre tu máquina y el mundo humano: interfaces web, dashboards de
          control y bases de datos en la nube. Es famoso por ser
          <strong>legible, ligero y basado en pares Clave:Valor</strong>.
        </TextBlock>
      </div>
    </section>

    <!-- 2. ANATOMÍA: CLAVE-VALOR -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Anatomía: El Mapa del Tesoro</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            JSON elimina las etiquetas redundantes. Todo se reduce a una estructura de diccionario:
            <br /><br />
            <strong>"Clave": Valor</strong>
          </TextBlock>

          <ul class="tool-list q-mt-md">
            <li>
              🔑 <strong>Comillas Dobles:</strong> Las claves <em>siempre</em> van entre
              <code>" "</code>. (Las simples <code>' '</code> rompen el formato).
            </li>
            <li>⛓️ <strong>Dos Puntos:</strong> Separan la clave del valor.</li>
            <li>
              📦 <strong>Tipos de Datos:</strong> Soporta Texto, Números, Booleanos, Listas
              <code>[]</code> y Objetos <code>{}</code>.
            </li>
          </ul>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL DISSECTION -->
          <div class="tool-card dissection-card relative-position q-pa-xl">
            <div class="font-mono text-h5">
              <span class="text-grey-6">{</span>
              <br />

              <!-- CLAVE -->
              <div class="q-pl-md row items-center wrap">
                <span class="text-yellow-4 text-weight-bold key-hover">"sensor_id"</span>
                <span class="text-grey-5 q-mx-sm">:</span>
                <span class="text-blue-4 value-hover">105</span><span class="text-grey-6">,</span>
              </div>

              <!-- LISTA -->
              <div class="q-pl-md row items-center q-mt-sm wrap">
                <span class="text-yellow-4 text-weight-bold key-hover">"posicion"</span>
                <span class="text-grey-5 q-mx-sm">:</span>
                <span class="text-purple-4 list-hover">[ 12.5, 40.1, 0.0 ]</span>
              </div>

              <span class="text-grey-6">}</span>
            </div>

            <!-- ANNOTATIONS -->
            <div class="annotations q-mt-xl relative-position row justify-around text-center">
              <!-- Key Label -->
              <div class="column items-center">
                <q-icon name="vpn_key" color="yellow-7" size="sm" />
                <div class="text-caption text-yellow-7 text-weight-bold">Clave (String)</div>
              </div>

              <!-- Value Label -->
              <div class="column items-center">
                <q-icon name="data_usage" color="blue-4" size="sm" />
                <div class="text-caption text-blue-4 text-weight-bold">Valor (Number)</div>
              </div>

              <!-- Array Label -->
              <div class="column items-center">
                <q-icon name="list" color="purple-4" size="sm" />
                <div class="text-caption text-purple-4 text-weight-bold">Array (Lista)</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. ESTRUCTURAS COMPLEJAS (NESTING) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Estructuras Anidadas</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Un robot no es plano. Tiene partes dentro de partes. JSON permite meter Objetos dentro
            de Objetos (Nesting).
            <br /><br />
            Observa cómo la propiedad <code>"bateria"</code> no es un simple número, sino un objeto
            completo con sus propios datos (voltaje, estado).
          </TextBlock>
          <div class="q-mt-lg">
            <AlertBlock type="warning" title="El error de la coma final">
              En JSON estándar, <strong>NO</strong> puedes poner una coma después del último
              elemento de una lista u objeto. Es el error de sintaxis #1.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <div class="tool-card bg-slate-900 q-pa-lg">
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="json"
              content='{
  "robot_name": "TurtleBot",
  "online": true,

  "bateria": {
    "voltaje": 12.4,
    "estado": "cargando",
    "celdas": [4.1, 4.1, 4.2]
  },

  "errores": null
}'
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. CASO REAL: ROSBRIDGE (WEB SOCKETS) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. En la Práctica: Rosbridge Protocol</SectionTitle>
      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-6">
          <TextBlock>
            ¿Cómo controlas un robot desde una página web? Usas un paquete llamado
            <strong>rosbridge</strong>. <br /><br />
            Este paquete convierte los mensajes complejos de C++ de ROS en texto JSON simple que
            cualquier navegador (Chrome/Firefox) puede entender.
          </TextBlock>
        </div>
        <div class="col-12 col-md-6">
          <div class="tool-card code-card q-pa-none border-yellow">
            <div
              class="bg-yellow-9 text-black q-px-md q-py-sm text-subtitle2 flex justify-between items-center border-bottom-light"
            >
              <span class="text-weight-bold">Mensaje WebSocket (Telemetría)</span>
              <q-icon name="wifi" />
            </div>
            <!-- CORREGIDO: lang & content -->
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
      "z": -1.2
    }
  }
}'
            />
            <div class="bg-dark-subtle q-px-md q-py-sm text-caption text-grey-5 border-top-light">
              👆 Este JSON le dice al robot: "Avanza a 0.5 m/s y gira a la derecha".
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. JSON vs PYTHON DICT (LA TRAMPA) -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>4. ¡Cuidado! JSON no es un Diccionario de Python</SectionTitle>
      <div class="tool-card comparison-card row items-center justify-center q-pa-lg">
        <div class="col-12 col-md-5 text-center">
          <div class="text-h6 text-green-4 q-mb-sm">Python (Dict)</div>
          <div class="bg-black q-pa-md rounded-borders font-mono text-body2 text-left border-green">
            {<br />
            <span class="text-green-4">'id'</span>: 1,
            <span class="text-grey-6 text-italic"># Comillas simples OK</span><br />
            <span class="text-green-4">True</span>,
            <span class="text-grey-6 text-italic"># T mayúscula</span><br />
            <span class="text-green-4">None</span>
            <span class="text-grey-6 text-italic"># N mayúscula</span><br />
            }
          </div>
        </div>

        <div class="col-12 col-md-2 text-center q-my-md">
          <div class="column items-center">
            <q-icon name="close" color="red" size="3rem" />
            <div class="text-caption text-red text-weight-bold">NO SON IGUALES</div>
          </div>
        </div>

        <div class="col-12 col-md-5 text-center">
          <div class="text-h6 text-yellow-4 q-mb-sm">JSON (String)</div>
          <div
            class="bg-black q-pa-md rounded-borders font-mono text-body2 text-left border-yellow"
          >
            {<br />
            <span class="text-yellow-4">"id"</span>: 1,
            <span class="text-red-4 text-italic"># Comillas DOBLES obligatorias</span><br />
            <span class="text-yellow-4">true</span>,
            <span class="text-red-4 text-italic"># minúscula</span><br />
            <span class="text-yellow-4">null</span>
            <span class="text-red-4 text-italic"># minúscula</span><br />
            }
          </div>
        </div>
      </div>
      <div class="text-center q-mt-lg text-grey-5">
        <q-icon name="info" color="primary" /> Para convertir entre ellos en Python, usa siempre
        <code>import json</code> y las funciones <code>json.loads()</code> /
        <code>json.dumps()</code>.
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
</script>

<style scoped>
/* --- ESTILOS MAESTROS --- */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(250, 204, 21, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  line-height: 1.1;
  color: #f8fafc;
}

/* TOOL CARDS */
.tool-card {
  height: 100%;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

/* DISSECTION CARD */
.dissection-card {
  border-top: 4px solid #facc15;
  background: #0f172a;
}

.key-hover:hover {
  text-shadow: 0 0 10px rgba(250, 204, 21, 0.6);
  cursor: help;
}
.value-hover:hover {
  text-shadow: 0 0 10px rgba(96, 165, 250, 0.6);
  cursor: help;
}
.list-hover:hover {
  text-shadow: 0 0 10px rgba(192, 132, 252, 0.6);
  cursor: help;
}

/* CODE CARD */
.tool-card.code-card {
  overflow: hidden;
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.tool-card.border-yellow {
  border-color: #facc15;
}

/* COMPARISON CARD */
.comparison-card {
  border: 1px dashed rgba(255, 255, 255, 0.2);
}
.border-green {
  border-left: 2px solid #4ade80;
}
.border-yellow {
  border-left: 2px solid #facc15;
}

/* UTILS */
.bg-dark-subtle {
  background: rgba(0, 0, 0, 0.3);
}
.bg-slate-900 {
  background: #0f172a;
}
.border-bottom-light {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.border-top-light {
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.tool-list {
  list-style: none;
  padding: 0;
  color: #cbd5e1;
}
.tool-list li {
  margin-bottom: 8px;
  font-size: 0.95rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .annotations {
    display: none;
  }
}
</style>
