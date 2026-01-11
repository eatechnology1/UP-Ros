<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-indigo-4 text-weight-bold q-mb-sm">
          MÓDULO 6.4: ACTUACIÓN VIRTUAL
        </div>
        <h1 class="hero-title">Plugins de Control: <span class="text-white">Los Músculos</span></h1>
        <TextBlock>
          Un modelo URDF es estático como una estatua. Para moverlo, necesitamos inyectar vida. Los
          **Plugins de Control** son el puente inverso: escuchan comandos de ROS 2 (como "avanza a 1
          m/s") y calculan las fuerzas físicas necesarias para girar las ruedas virtuales en Gazebo.
          Sin ellos, tu robot es solo un pisapapeles muy caro.
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. El Chofer Fantasma</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            En un robot real, tienes drivers de motor (hardware) que envían corriente eléctrica. En
            Gazebo, usamos el plugin <code>libgazebo_ros_diff_drive.so</code>. <br /><br />
            Este plugin hace dos trabajos críticos:
          </TextBlock>
          <ul class="q-pl-md text-grey-4 tool-list">
            <li>📥 <strong>Suscribir:</strong> Escucha <code>/cmd_vel</code> y gira las ruedas.</li>
            <li>
              📤 <strong>Publicar:</strong> Calcula cuánto se ha movido el robot (Odometría) y lo
              cuenta a ROS.
            </li>
          </ul>
        </template>
        <template #right>
          <div
            class="tool-card bg-slate-900 flex flex-center relative-position overflow-hidden h-full"
          >
            <div class="row items-center justify-around full-width">
              <div class="column items-center">
                <div class="cmd-box bg-blue-9 shadow-blue q-pa-sm rounded-borders text-center">
                  <div class="text-caption text-white font-mono">/cmd_vel</div>
                  <q-icon name="keyboard_arrow_up" color="white" />
                </div>
              </div>

              <div class="plugin-gear relative-position">
                <q-icon name="settings" size="4rem" color="indigo-4" class="spin-gear" />
              </div>

              <div class="column items-center">
                <div class="wheel-group row q-gutter-xs">
                  <div class="wheel bg-grey-5 spin-fast"></div>
                  <div class="wheel bg-grey-5 spin-fast"></div>
                </div>
                <div class="text-caption text-grey-5 q-mt-xs">Torque Physics</div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. Cinemática Diferencial (El Tanque)</SectionTitle>
      <TextBlock>
        La mayoría de robots educativos usan <strong>Differential Drive</strong>. Dos ruedas
        motrices y una loca (caster). Para girar, no mueven un volante; cambian la velocidad
        relativa de las ruedas.
      </TextBlock>

      <div class="row q-col-gutter-lg q-mt-md">
        <div class="col-12 col-md-4">
          <div class="custom-card border-green q-pa-md text-center">
            <div class="text-h6 text-green-3">Avanzar</div>
            <div class="text-caption text-grey-4">Izquierda == Derecha</div>
            <div class="arrow-up q-mx-auto q-my-sm"></div>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="custom-card border-yellow q-pa-md text-center">
            <div class="text-h6 text-yellow-3">Girar Izquierda</div>
            <div class="text-caption text-grey-4">Der > Izq</div>
            <div class="arrow-curve-left q-mx-auto q-my-sm"></div>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="custom-card border-red q-pa-md text-center">
            <div class="text-h6 text-red-3">Giro en Sitio</div>
            <div class="text-caption text-grey-4">Der = -Izq</div>
            <div class="spin-icon q-mx-auto q-my-sm"></div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. geometry_msgs/Twist</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Para mover cualquier robot en ROS (dron, barco, coche), usamos el mismo mensaje. Se
            compone de dos vectores 3D:
          </TextBlock>
          <ul class="q-pl-md text-grey-4 tool-list">
            <li>🚀 <strong>Linear:</strong> Metros/segundo. (Usamos X para avanzar).</li>
            <li>🔄 <strong>Angular:</strong> Radianes/segundo. (Usamos Z para girar).</li>
          </ul>
        </template>
        <template #right>
          <div class="tool-card bg-black q-pa-md font-mono text-sm border-indigo shadow-indigo">
            <div class="text-grey-5"># Estructura del mensaje</div>
            <div class="q-my-sm">
              <span class="text-blue-4">linear:</span><br />
              &nbsp;&nbsp;x: <span class="text-green-4">0.5</span>
              <span class="text-grey-6 text-italic"># Avanza</span><br />
              &nbsp;&nbsp;y: 0.0<br />
              &nbsp;&nbsp;z: 0.0
            </div>
            <div>
              <span class="text-purple-4">angular:</span><br />
              &nbsp;&nbsp;x: 0.0<br />
              &nbsp;&nbsp;y: 0.0<br />
              &nbsp;&nbsp;z: <span class="text-yellow-4">1.57</span>
              <span class="text-grey-6 text-italic"># Gira Izq</span>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>4. Implementación en URDF/Xacro</SectionTitle>
      <TextBlock>
        Copia este bloque dentro de tu etiqueta <code>&lt;robot&gt;</code>. Asegúrate de que los
        nombres de los <code>joints</code> coincidan con los tuyos.
      </TextBlock>

      <CodeBlock
        lang="xml"
        title="diff_drive.xacro"
        code='<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.35</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>

    <max_wheel_torque>200</max_wheel_torque>
    <max_wheel_acceleration>10.0</max_wheel_acceleration>

    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>true</publish_wheel_tf>
  </plugin>
</gazebo>'
      />
      <AlertBlock type="danger" title="Error Común: publish_odom_tf">
        Si pones <code>publish_odom_tf</code> en <strong>true</strong> y también tienes un nodo
        <code>robot_localization</code> (EKF), ambos pelearán por publicar la transformación. Solo
        uno puede ser el dueño de la verdad.
      </AlertBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>5. Odometría: La Ilusión de Posición</SectionTitle>
      <TextBlock>
        El plugin cuenta cuántas vueltas dieron las ruedas y estima dónde está el robot (x, y,
        theta).
        <br />
        En simulación es perfecta (a menos que añadas ruido). En la realidad, las ruedas patinan y
        la odometría "deriva" (drift).
      </TextBlock>

      <div
        class="tool-card bg-slate-900 q-pa-lg relative-position overflow-hidden q-mt-md"
        style="height: 200px"
      >
        <div class="absolute-top-left q-ma-md text-caption text-grey-5">
          Trayectoria Real vs Odometría
        </div>

        <svg class="full-width full-height absolute">
          <path
            d="M 50 150 Q 150 150, 250 100 T 450 50"
            fill="none"
            stroke="#4ade80"
            stroke-width="3"
            stroke-dasharray="5,5"
          />
          <path
            d="M 50 150 Q 150 145, 260 110 T 460 80"
            fill="none"
            stroke="#f87171"
            stroke-width="3"
          />
        </svg>

        <div class="legend row absolute-bottom-right q-ma-md bg-black q-pa-xs rounded">
          <div class="row items-center q-mr-md">
            <div class="dot bg-green-4 q-mr-xs"></div>
            <span class="text-xxs text-grey-4">Realidad</span>
          </div>
          <div class="row items-center">
            <div class="dot bg-red-4 q-mr-xs"></div>
            <span class="text-xxs text-grey-4">Lo que el robot cree</span>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>6. Joint States Publisher</SectionTitle>
      <TextBlock>
        Para que RViz sepa dónde dibujar las ruedas, Gazebo debe reportar el ángulo actual de
        rotación. El plugin <code>diff_drive</code> se encarga de publicar en
        <code>/joint_states</code>.
      </TextBlock>
      <CodeBlock lang="bash" code="ros2 topic echo /joint_states" />
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>7. Teleop: Conduciendo Manualmente</SectionTitle>
      <TextBlock>
        No escribas código para probar. Usa el nodo estándar de teclado para enviar mensajes Twist.
      </TextBlock>

      <div class="tool-card cli-card bg-black q-pa-md border-indigo shadow-indigo">
        <div class="text-grey-6 font-mono text-xs q-mb-sm">
          $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
        </div>

        <div class="keyboard-grid q-mx-auto" style="width: 150px">
          <div class="row justify-center">
            <div class="key bg-slate-800 text-white border-light">i</div>
          </div>
          <div class="row justify-between q-mt-xs">
            <div class="key bg-slate-800 text-white border-light">j</div>
            <div class="key bg-slate-800 text-white border-light">k</div>
            <div class="key bg-slate-800 text-white border-light">l</div>
          </div>
        </div>

        <div class="text-center text-indigo-3 q-mt-md font-italic text-caption">
          "Usa las teclas I-J-K-L para moverte"
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>8. Si las ruedas patinan...</SectionTitle>
      <AlertBlock type="warning" title="Física de Contacto">
        Si tus ruedas giran pero el robot no avanza, probablemente tienes fricción cero. Debes
        definir los coeficientes <code>mu1</code> y <code>mu2</code> en la etiqueta
        <code>&lt;gazebo&gt;</code> de las ruedas.
      </AlertBlock>
      <CodeBlock
        lang="xml"
        code='<gazebo reference="left_wheel">
  <mu1>1.0</mu1> <mu2>1.0</mu2> <kp>500000.0</kp> <kd>10.0</kd>     </gazebo>'
      />
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>9. Más allá del Diferencial</SectionTitle>
      <TextBlock> No todos los robots son tanques. Gazebo tiene plugins para todo: </TextBlock>
      <div class="row q-gutter-sm q-mt-sm">
        <q-chip icon="directions_car" color="purple-9" text-color="white">Ackermann (Coche)</q-chip>
        <q-chip icon="apps" color="purple-9" text-color="white">Omnidireccional (Mecanum)</q-chip>
        <q-chip icon="flight" color="purple-9" text-color="white">Drones (Rotores)</q-chip>
      </div>
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>10. Diagnóstico de Motores</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12">
          <q-expansion-item
            icon="swap_horiz"
            label="El robot avanza al revés"
            header-class="bg-slate-800 text-white rounded-borders"
            class="q-mb-sm border-light"
          >
            <div class="q-pa-md bg-slate-900 text-grey-4">
              Probablemente el eje Z de tus ruedas en el URDF está apuntando hacia adentro en lugar
              de hacia afuera. En lugar de rotar la malla, rota el
              <code>&lt;axis xyz="0 1 0"/&gt;</code> en el Joint.
            </div>
          </q-expansion-item>

          <q-expansion-item
            icon="report_problem"
            label="El robot tiembla violentamente"
            header-class="bg-slate-800 text-white rounded-borders"
            class="q-mb-sm border-light"
          >
            <div class="q-pa-md bg-slate-900 text-grey-4">
              Conflicto de masas. Si el chasis pesa 0.1kg y las ruedas 5kg, el motor de física se
              vuelve inestable. Asegúrate de que las masas sean realistas y proporcionales.
            </div>
          </q-expansion-item>
        </div>
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
/* GENERAL */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}
.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(99, 102, 241, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
  margin-bottom: 3rem;
}
.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  color: #f8fafc;
}

/* CARDS */
.tool-card {
  border-radius: 12px;
  border-left: 4px solid;
}
.custom-card {
  border-radius: 12px;
  background: rgba(30, 41, 59, 0.5);
  border-top: 4px solid;
  height: 100%;
}
.shadow-blue {
  box-shadow: 0 0 15px rgba(59, 130, 246, 0.4);
}
.shadow-indigo {
  box-shadow: 0 0 20px rgba(99, 102, 241, 0.2);
}

.border-green {
  border-color: #4ade80;
}
.border-yellow {
  border-color: #facc15;
}
.border-red {
  border-color: #f87171;
}
.border-indigo {
  border-color: #6366f1;
}
.text-indigo-4 {
  color: #818cf8;
}

/* ANIMATIONS */
.spin-gear {
  animation: spin 4s infinite linear;
}
.spin-fast {
  animation: spin 1s infinite linear;
}
.wheel {
  width: 15px;
  height: 30px;
  border-radius: 4px;
  border: 1px solid #333;
}
@keyframes spin {
  from {
    transform: rotate(0deg);
  }
  to {
    transform: rotate(360deg);
  }
}

/* DIFF DRIVE ARROWS */
.arrow-up {
  width: 0;
  height: 0;
  border-left: 10px solid transparent;
  border-right: 10px solid transparent;
  border-bottom: 20px solid #4ade80;
}
.arrow-curve-left {
  width: 40px;
  height: 40px;
  border-top: 5px solid #facc15;
  border-left: 5px solid #facc15;
  border-radius: 100% 0 0 0;
}
.spin-icon {
  width: 30px;
  height: 30px;
  border: 3px solid #f87171;
  border-radius: 50%;
  border-top-color: transparent;
  animation: spin 2s infinite linear;
}

/* KEYBOARD */
.key {
  width: 40px;
  height: 40px;
  border-radius: 6px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: bold;
  font-family: monospace;
  box-shadow: 0 4px 0 rgba(255, 255, 255, 0.1);
}
.key:active {
  transform: translateY(4px);
  box-shadow: none;
}

/* ODOM DRIFT */
.dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
}

/* UTILS */
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xxs {
  font-size: 0.7rem;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
</style>
