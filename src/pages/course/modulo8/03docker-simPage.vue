<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-light-blue-4 text-weight-bold q-mb-sm">
          MÓDULO 8.3: CONTENERIZACIÓN
        </div>
        <h1 class="hero-title">Docker: El <span class="text-white">Traje Espacial</span></h1>
        <TextBlock>
          ¿Te ha pasado que tu código funciona perfecto en tu casa pero falla en la demo por una
          "dependencia faltante"?
          <br /><br />
          <strong>Docker</strong> soluciona el problema de <em>"Funciona en mi máquina"</em>.
          Empaqueta tu código, tus librerías, tus configuraciones e incluso el sistema operativo
          entero en un bloque inmutable llamado <strong>Contenedor</strong>. Si corre en tu Docker,
          corre en cualquier parte.
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. ¿Por qué no usar una Máquina Virtual?</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Las Máquinas Virtuales (VM) son pesadas. Emulan hardware completo. Arrancar una VM toma
            minutos y consume GBs de RAM.
            <br /><br />
            Los <strong>Contenedores</strong> son ligeros. Comparten el núcleo (Kernel) del sistema
            anfitrión pero aíslan todo lo demás. Arrancan en milisegundos.
            <br />
            Es la diferencia entre construir una casa nueva para cada invitado (VM) o simplemente
            darles una habitación privada en tu casa (Docker).
          </TextBlock>
        </template>
        <template #right>
          <div class="tool-card bg-slate-900 q-pa-md h-full flex flex-center">
            <div class="row q-gutter-md justify-center items-end full-width">
              <div class="column items-center">
                <div class="stack-box bg-red-4">App A</div>
                <div class="stack-box bg-red-3">Libs</div>
                <div class="stack-box bg-red-2 text-black">Guest OS</div>
                <div class="stack-box bg-grey-8" style="height: 10px">Hypervisor</div>
                <div class="text-caption text-red-2 q-mt-xs">VM (Pesado)</div>
              </div>

              <div class="column items-center">
                <div class="stack-box bg-light-blue-4">App A</div>
                <div class="stack-box bg-light-blue-3">Libs</div>
                <div class="stack-box bg-transparent border-dashed text-grey-5">No OS</div>
                <div class="stack-box bg-blue-9 text-white">Docker Engine</div>
                <div class="text-caption text-light-blue-2 q-mt-xs">Container (Ligero)</div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. La Receta (Image) vs El Pastel (Container)</SectionTitle>
      <TextBlock> Concepto clave: </TextBlock>
      <div class="row q-col-gutter-lg q-mt-md">
        <div class="col-12 col-md-6">
          <div class="custom-card border-blue q-pa-md">
            <div class="row items-center q-mb-sm">
              <q-icon name="save" color="blue-4" size="md" class="q-mr-sm" />
              <div class="text-h6 text-blue-1">Imagen (Image)</div>
            </div>
            <p class="text-grey-4 text-caption">
              Es el archivo estático. La "foto" del sistema. Es de solo lectura.
              <br />Ejemplo: <code>osrf/ros:humble-desktop</code>
            </p>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="custom-card border-green q-pa-md">
            <div class="row items-center q-mb-sm">
              <q-icon name="play_circle" color="green-4" size="md" class="q-mr-sm" />
              <div class="text-h6 text-green-1">Contenedor (Container)</div>
            </div>
            <p class="text-grey-4 text-caption">
              Es la instancia viva de la imagen. Puedes tener 10 contenedores corriendo la misma
              imagen. Cuando lo apagas, los cambios se pierden (si no usas volúmenes).
            </p>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. Escribiendo el Dockerfile</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Para crear tu imagen personalizada, escribes un archivo de texto con instrucciones paso
            a paso.
          </TextBlock>
          <ul class="q-pl-md text-grey-4 tool-list font-mono text-xs">
            <li><span class="text-purple-4">FROM</span>: La base (Ubuntu + ROS ya instalados).</li>
            <li>
              <span class="text-blue-4">RUN</span>: Comandos de terminal (instalar dependencias).
            </li>
            <li><span class="text-green-4">COPY</span>: Meter tu código dentro.</li>
            <li><span class="text-yellow-4">CMD</span>: Qué hacer al arrancar.</li>
          </ul>
        </template>
        <template #right>
          <CodeBlock
            lang="dockerfile"
            title="Dockerfile"
            code="# 1. Usar imagen oficial de ROS 2
FROM osrf/ros:humble-desktop

# 2. Instalar herramientas extra
RUN apt-get update && apt-get install -y \
    python3-pip \
    nano \
    && rm -rf /var/lib/apt/lists/*

# 3. Crear workspace
WORKDIR /ros2_ws
COPY ./src ./src

# 4. Compilar
RUN . /opt/ros/humble/setup.sh && colcon build

# 5. Configurar entorno al inicio
COPY entrypoint.sh /
ENTRYPOINT ['/entrypoint.sh']
CMD ['bash']"
          />
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>4. Volúmenes: Persistencia de Datos</SectionTitle>
      <AlertBlock type="warning" title="El Agujero Negro">
        Los contenedores son <strong>efímeros</strong>. Si editas un archivo dentro del contenedor y
        luego lo borras, ese archivo desaparece para siempre.
        <br />
        Para desarrollar, usamos <strong>Volúmenes</strong> (Bind Mounts): conectamos una carpeta de
        tu PC real a una carpeta dentro del contenedor. Es como un portal mágico.
      </AlertBlock>

      <div
        class="tool-card bg-black q-pa-lg q-mt-md relative-position overflow-hidden"
        style="height: 200px"
      >
        <div class="row justify-between items-center h-full q-px-xl">
          <div class="column items-center z-top">
            <q-icon name="laptop" size="4rem" color="grey-5" />
            <div class="text-caption text-white">Tu Laptop</div>
            <div class="folder bg-blue-9 q-pa-xs text-xxs rounded">/mi_codigo</div>
          </div>

          <div class="pipe-connection relative-position">
            <div class="data-packet"></div>
          </div>

          <div class="column items-center z-top">
            <div class="docker-box border-blue q-pa-md">
              <q-icon name="inventory_2" size="3rem" color="light-blue-4" />
            </div>
            <div class="text-caption text-white">Container</div>
            <div class="folder bg-blue-9 q-pa-xs text-xxs rounded">/root/ws/src</div>
          </div>
        </div>
      </div>

      <div class="code-label bash q-mt-md">Comando con Volumen</div>
      <CodeBlock lang="bash" code="docker run -it -v $(pwd)/src:/ros2_ws/src my_robot_image" />
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>5. El Reto: Gráficos y Redes</SectionTitle>
      <TextBlock>
        Docker es genial para servidores, pero en robótica necesitamos ver <strong>RViz</strong> y
        <strong>Gazebo</strong>. Por defecto, Docker no tiene acceso a tu pantalla ni a tu tarjeta
        gráfica.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="tool-card bg-slate-900 border-red q-pa-md">
            <div class="text-h6 text-red-3">Sin Configuración</div>
            <div class="text-caption text-grey-4">
              $ ros2 run rviz2<br />
              <span class="text-red-5">Error: Could not connect to display.</span>
            </div>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="tool-card bg-slate-900 border-green q-pa-md">
            <div class="text-h6 text-green-3">La Solución: Rocker</div>
            <div class="text-caption text-grey-4">
              Usamos una herramienta llamada <strong>Rocker</strong> o pasamos flags especiales para
              compartir el X11 server.
            </div>
          </div>
        </div>
      </div>

      <CodeBlock
        title="Comando mágico para GUI"
        lang="bash"
        code="docker run -it \
    --net=host \
    --env='DISPLAY' \
    --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw' \
    osrf/ros:humble-desktop \
    rviz2"
      />
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>6. Docker Compose: Lanzar Flotas</SectionTitle>
      <TextBlock>
        ¿Recuerdas los Launch Files para nodos? <strong>Docker Compose</strong> es lo mismo pero
        para contenedores. Puedes definir un contenedor para el simulador, otro para el path
        planning y otro para la base de datos, y lanzarlos todos juntos.
      </TextBlock>

      <CodeBlock
        lang="yaml"
        title="docker-compose.yml"
        code="version: '3'
services:
  simulacion:
    image: my_robot_sim
    volumes:
      - ./src:/ws/src
    environment:
      - DISPLAY=$DISPLAY

  navegacion:
    image: nav2_stack
    depends_on:
      - simulacion"
      />
      <div class="text-center q-mt-sm">
        <q-chip color="light-blue-9" text-color="white" icon="play_arrow">docker compose up</q-chip>
      </div>
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>7. Del Laptop al Robot</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Aquí está la magia final.
            <ol>
              <li>
                Construyes la imagen en tu PC potente: <code>docker build -t mi_robot:v1 .</code>
              </li>
              <li>La subes a la nube (Docker Hub).</li>
              <li>
                En el robot real (Raspberry Pi/Jetson), solo escribes:
                <code>docker pull mi_robot:v1</code>.
              </li>
            </ol>
            ¡Y listo! No tienes que instalar ROS, ni librerías, ni compilar nada en el robot.
          </TextBlock>
        </template>
        <template #right>
          <div class="tool-card bg-white relative-position overflow-hidden h-full flex flex-center">
            <div class="cloud-upload">
              <q-icon name="cloud_upload" size="4rem" color="light-blue-5" class="floating" />
            </div>
            <div class="path-line"></div>
            <div class="robot-target">
              <q-icon name="smart_toy" size="3rem" color="black" />
            </div>
          </div>
        </template>
      </SplitBlock>
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
    radial-gradient(circle at center, rgba(3, 169, 244, 0.15), transparent 60%),
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

/* VISUALIZATIONS */
.tool-card {
  border-radius: 12px;
  height: 100%;
}
.custom-card {
  border-radius: 12px;
  background: rgba(30, 41, 59, 0.5);
  border-left: 4px solid;
  height: 100%;
}
.h-full {
  height: 100%;
}

/* VM VS DOCKER STACKS */
.stack-box {
  width: 120px;
  padding: 4px;
  margin-bottom: 2px;
  text-align: center;
  font-size: 0.7rem;
  font-weight: bold;
  border-radius: 2px;
}
.border-dashed {
  border: 1px dashed #666;
}

/* VOLUME PIPE ANIMATION */
.pipe-connection {
  flex-grow: 1;
  height: 10px;
  background: #334155;
  margin: 0 10px;
  border-radius: 5px;
  position: relative;
}
.data-packet {
  width: 20px;
  height: 14px;
  background: #4fc3f7;
  border-radius: 4px;
  position: absolute;
  top: -2px;
  left: 0;
  animation: dataFlow 2s infinite linear;
}
@keyframes dataFlow {
  0% {
    left: 0;
  }
  50% {
    left: calc(100% - 20px);
  }
  100% {
    left: 0;
  }
}
.z-top {
  z-index: 5;
}
.docker-box {
  border-radius: 8px;
  border: 2px solid #03a9f4;
}
.folder {
  margin-top: 5px;
}

/* DEPLOY ANIMATION */
.floating {
  animation: float 3s infinite ease-in-out;
}
@keyframes float {
  0%,
  100% {
    transform: translateY(0);
  }
  50% {
    transform: translateY(-10px);
  }
}

/* UTILS */
.bg-slate-900 {
  background: #0f172a;
}
.text-xxs {
  font-size: 0.6rem;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-blue {
  border-color: #03a9f4;
}
.border-green {
  border-color: #4ade80;
}
.border-red {
  border-color: #f44336;
}
.text-light-blue-4 {
  color: #29b6f6;
}
.bg-light-blue-4 {
  background: #29b6f6;
}
.bg-light-blue-3 {
  background: #4fc3f7;
}
</style>
