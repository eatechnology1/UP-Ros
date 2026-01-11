<template>
  <LessonContainer>
    <div class="section-group self-stretch">
      <SectionTitle>1. La Regla de Oro: No Hardcoding</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El "Hardcoding" (escribir valores fijos en el código) es el enemigo de la escalabilidad.
            <br /><br />
            <strong>La Filosofía ROS 2:</strong>
            El nodo declara <em>qué</em> parámetros necesita ("necesito una velocidad máxima"). El
            archivo YAML provee el <em>valor</em> ("es 2.5 m/s").
          </TextBlock>
        </template>
        <template #right>
          <div
            class="tool-card bg-slate-900 flex flex-center full-height relative-position overflow-hidden border-orange"
          >
            <div class="row items-center justify-around full-width q-px-md">
              <div class="column items-center">
                <div
                  class="box-lock bg-grey-9 q-pa-md rounded-borders border-light text-center shadow-2"
                >
                  <q-icon name="lock" color="grey-5" size="2.5rem" />
                  <div class="text-caption text-grey-5 q-mt-sm font-mono text-xs">binary_code</div>
                </div>
              </div>

              <div class="arrow-dashed text-grey-6">
                <q-icon name="arrow_back" size="2rem" />
              </div>

              <div class="column items-center">
                <div
                  class="box-yaml bg-deep-orange-9 q-pa-md rounded-borders shadow-orange text-center cursor-pointer hover-pulse transition-hover"
                >
                  <q-icon name="tune" color="white" size="2.5rem" />
                  <div class="text-caption text-white q-mt-sm font-mono text-xs">params.yaml</div>
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. Anatomía del Archivo .yaml</SectionTitle>
      <TextBlock>
        La estructura es estricta. Si te saltas la clave mágica <code>ros__parameters</code>, ROS
        ignorará todo.
      </TextBlock>

      <div class="row q-col-gutter-lg q-mt-md">
        <div class="col-12 col-md-6">
          <CodeBlock
            lang="yaml"
            title="config/my_robot.yaml"
            code="my_node_name:           # 1. Nombre del Nodo
  ros__parameters:      # 2. LA CLAVE MÁGICA
    max_speed: 2.5      # Float
    waypoints: [1, 2, 3] # Lista
    use_lidar: true     # Bool

    # Parámetros anidados
    pid:
      p: 1.0
      i: 0.01
      d: 0.5"
            :copyable="true"
          />
        </div>
        <div class="col-12 col-md-6">
          <AlertBlock type="danger" title="Indentación (Espacios)">
            YAML odia los tabuladores (Tabs). Usa siempre <strong>2 espacios</strong> por nivel. Un
            espacio de más o de menos romperá el archivo silenciosamente.
          </AlertBlock>
          <AlertBlock type="info" title="Namespaces">
            Si lanzas el nodo dentro de un namespace (ej: <code>/robot1/my_node</code>), el YAML
            debe reflejarlo o usar wildcards (<code>/**</code>).
          </AlertBlock>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. El Comodín Global (/**)</SectionTitle>
      <TextBlock>
        A veces quieres configurar 10 nodos iguales con el mismo parámetro (ej: todos usan
        <code>use_sim_time: true</code>). En lugar de escribir el nodo 10 veces, usamos el comodín
        doble asterisco.
      </TextBlock>

      <CodeBlock
        lang="yaml"
        title="global_params.yaml"
        code="/**:                    # Aplica a CUALQUIER nodo
  ros__parameters:
    use_sim_time: true    # Vital para Gazebo"
        :copyable="true"
      />
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>4. Inyectando el YAML en el Launch</SectionTitle>
      <TextBlock>
        El archivo YAML no hace nada por sí solo. Debes pasárselo al nodo al nacer dentro del Launch
        File.
      </TextBlock>
      <CodeBlock
        lang="python"
        title="bringup_launch.py"
        code="config = os.path.join(
    get_package_share_directory('my_bot'),
    'config',
    'my_params.yaml'
)

Node(
    package='my_bot',
    executable='controller',
    parameters=[config]  # <--- AQUÍ SE INYECTA
)"
        :copyable="true"
      />
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>5. Reconfiguración en Caliente</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Lo más poderoso de ROS 2:
            <strong>Puedes cambiar los parámetros mientras el robot corre.</strong> <br /><br />
            Esto es ideal para sintonizar controladores PID (Sintonización fina). No apagues el
            robot. Simplemente inyecta el nuevo valor.
          </TextBlock>
          <div class="code-label bash q-mt-md">Terminal</div>
          <CodeBlock lang="bash" code="ros2 param set /my_node max_speed 5.0" :copyable="true" />
        </template>

        <template #right>
          <div
            class="tool-card bg-black flex flex-center full-height relative-position overflow-hidden border-orange"
          >
            <div class="absolute-top full-width" style="height: 70%">
              <div
                class="robot-speed-icon bg-blue-5 shadow-blue absolute transition-linear"
                :style="{ left: speedPos + '%' }"
              >
                <q-icon name="smart_toy" color="white" size="xs" />
              </div>
              <div class="speed-lines" :style="{ opacity: (speedVal - 1) / 4 }"></div>
            </div>

            <div class="absolute-bottom bg-slate-900 full-width q-pa-md border-top-light">
              <div
                class="row items-center justify-between text-caption text-grey-4 q-mb-xs font-mono"
              >
                <span>/my_node: max_speed</span>
                <span class="text-deep-orange-4 text-weight-bold"
                  >{{ speedVal.toFixed(1) }} m/s</span
                >
              </div>

              <div
                class="slider-track bg-grey-8 rounded-borders relative-position overflow-hidden"
                style="height: 6px"
              >
                <div
                  class="slider-fill bg-deep-orange-5"
                  :style="{ width: ((speedVal - 1) / 4) * 100 + '%' }"
                ></div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>6. Herramientas CLI: ros2 param</SectionTitle>
      <TextBlock> Domina estos comandos y dominarás el sistema. </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <q-list bordered separator class="bg-slate-900 rounded-borders border-orange-dim">
            <q-item>
              <q-item-section>
                <q-item-label class="text-deep-orange-4 font-mono font-bold">list</q-item-label>
                <q-item-label caption class="text-grey-4"
                  >Muestra todos los parámetros disponibles.</q-item-label
                >
              </q-item-section>
            </q-item>
            <q-item>
              <q-item-section>
                <q-item-label class="text-deep-orange-4 font-mono font-bold">get</q-item-label>
                <q-item-label caption class="text-grey-4">Lee el valor actual.</q-item-label>
                <div class="bg-black q-pa-xs rounded text-grey-5 font-mono text-xxs q-mt-xs">
                  $ ros2 param get /node param
                </div>
              </q-item-section>
            </q-item>
          </q-list>
        </div>
        <div class="col-12 col-md-6">
          <q-list bordered separator class="bg-slate-900 rounded-borders border-orange-dim">
            <q-item>
              <q-item-section>
                <q-item-label class="text-deep-orange-4 font-mono font-bold">set</q-item-label>
                <q-item-label caption class="text-grey-4"
                  >Escribe un nuevo valor en vivo.</q-item-label
                >
              </q-item-section>
            </q-item>
            <q-item>
              <q-item-section>
                <q-item-label class="text-deep-orange-4 font-mono font-bold">dump</q-item-label>
                <q-item-label caption class="text-grey-4">Guarda la config a archivo.</q-item-label>
                <div class="bg-black q-pa-xs rounded text-grey-5 font-mono text-xxs q-mt-xs">
                  $ ros2 param dump /node > save.yaml
                </div>
              </q-item-section>
            </q-item>
          </q-list>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>7. El Contrato en el Código</SectionTitle>
      <TextBlock>
        Para que un nodo acepte un parámetro del YAML, debe <strong>declararlo</strong> primero. Si
        intentas pasar un parámetro no declarado, ROS 2 te dará error.
      </TextBlock>

      <div class="row q-col-gutter-lg q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="code-label python">Python</div>
          <CodeBlock
            lang="python"
            code="self.declare_parameter('max_speed', 1.0)
speed = self.get_parameter('max_speed').value"
            :copyable="true"
          />
        </div>
        <div class="col-12 col-md-6">
          <div class="code-label cpp">C++</div>
          <CodeBlock
            lang="cpp"
            code="this->declare_parameter('max_speed', 1.0);
double speed = this->get_parameter('max_speed').as_double();"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>8. Mi parámetro no se carga</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12">
          <q-expansion-item
            icon="key_off"
            label="Error: Parameter not declared"
            header-class="bg-slate-800 text-white rounded-borders border-light"
            class="q-mb-sm shadow-1"
          >
            <div class="q-pa-md bg-slate-900 text-grey-4 border-light border-top-0">
              Estás intentando cargar <code>my_param</code> desde el YAML, pero en tu código
              Python/C++ olvidaste poner <code>self.declare_parameter('my_param')</code>.
            </div>
          </q-expansion-item>

          <q-expansion-item
            icon="visibility_off"
            label="El nodo arranca pero ignora el YAML"
            header-class="bg-slate-800 text-white rounded-borders border-light"
            class="q-mb-sm shadow-1"
          >
            <div class="q-pa-md bg-slate-900 text-grey-4 border-light border-top-0">
              Revisa 3 cosas:<br />
              1. ¿Olvidaste la clave <code>ros__parameters</code>?<br />
              2. ¿El nombre del nodo en el YAML coincide EXACTAMENTE con el nombre del nodo en el
              Launch?<br />
              3. ¿Estás usando namespaces?
            </div>
          </q-expansion-item>
        </div>
      </div>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';

// Animation Logic for Slider
const speedVal = ref(1.0);
const speedPos = ref(0);
let direction = 1;

let interval: ReturnType<typeof setInterval> | undefined;

onMounted(() => {
  interval = setInterval(() => {
    // Oscillate speed value smoothly
    if (speedVal.value >= 5.0) direction = -1;
    if (speedVal.value <= 1.0) direction = 1;
    speedVal.value += 0.05 * direction;

    // Update robot position faster based on CURRENT speed
    speedPos.value += speedVal.value * 0.4;
    if (speedPos.value > 110) speedPos.value = -10; // Wrap around
  }, 50);
});

onUnmounted(() => {
  if (interval) clearInterval(interval);
});
</script>

<style scoped>
/* GENERAL */
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

/* CARDS */
.tool-card {
  height: 100%;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

.full-height {
  height: 100%;
}
.full-width {
  width: 100%;
}

/* ANIMATION BOXES */
.box-lock,
.box-yaml {
  width: 100px;
  transition: all 0.3s;
}
.hover-pulse:hover {
  transform: scale(1.05);
  box-shadow: 0 0 20px rgba(255, 87, 34, 0.6);
}
.shadow-orange {
  box-shadow: 0 0 15px rgba(255, 87, 34, 0.4);
}
.transition-hover {
  transition: transform 0.2s;
}

/* ROBOT & SLIDER ANIMATION */
.robot-speed-icon {
  width: 32px;
  height: 32px;
  border-radius: 6px;
  display: flex;
  align-items: center;
  justify-content: center;
  top: 40%;
  z-index: 10;
}
.shadow-blue {
  box-shadow: 0 0 15px rgba(33, 150, 243, 0.5);
}

.speed-lines {
  position: absolute;
  top: 40%;
  left: 0;
  width: 100%;
  height: 32px;
  background: repeating-linear-gradient(
    90deg,
    transparent,
    transparent 20px,
    rgba(255, 255, 255, 0.1) 20px,
    rgba(255, 255, 255, 0.1) 40px
  );
  z-index: 1;
}

.slider-fill {
  height: 100%;
  transition: width 0.1s linear;
}

/* UTILS */
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.text-deep-orange-4 {
  color: #ff7043;
}
.bg-deep-orange-9 {
  background: #bf360c;
}
.bg-deep-orange-5 {
  background: #ff5722;
}
.border-orange {
  border-left: 4px solid #ff5722;
}
.border-orange-dim {
  border: 1px solid rgba(255, 87, 34, 0.3);
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-top-light {
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}
.border-top-0 {
  border-top: 0;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.8rem;
}
.text-xxs {
  font-size: 0.7rem;
}
.font-bold {
  font-weight: 700;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
