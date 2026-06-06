<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        Mucha gente cree que ROS es un Sistema Operativo (como Windows o Linux). **Error.**
        <br /><br />
        ROS (Robot Operating System) es en realidad un **Middleware**. Es el "pegamento" de software
        que permite que un sensor láser fabricado en Alemania hable con un motor fabricado en Japón,
        usando un algoritmo escrito por ti en Python.
        <br /><br />
        En ROS 2, este pegamento tiene un nombre industrial:
        <strong>DDS (Data Distribution Service)</strong>.
      </TextBlock>
    </div>

    <!-- 1. ARQUITECTURA: ROS 1 VS ROS 2 -->
    <div class="section-group">
      <SectionTitle>1. El Cambio de Paradigma: Adiós al "Master"</SectionTitle>
      <TextBlock> Para entender ROS 2, primero debemos matar al fantasma de ROS 1. </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="text-h6 text-negative q-mb-sm">ROS 1 (Centralizado)</div>
          <TextBlock>
            Necesitaba un <strong>Master Node</strong> (`roscore`) que actuaba como operadora
            telefónica.
            <br />
            Si el Master se caía, <strong>todo el robot moría</strong>. Los nodos no sabían hablar
            entre sí directamente.
          </TextBlock>
          <div class="architecture-box ros1">
            <div class="box-node">Nodo A</div>
            <div class="box-master">MASTER</div>
            <div class="box-node">Nodo B</div>
            <div class="arrow-up">⬆</div>
            <div class="arrow-up">⬆</div>
          </div>
        </template>
        <template #right>
          <div class="text-h6 text-positive q-mb-sm">ROS 2 (Distribuido)</div>
          <TextBlock>
            No existe Master. Usamos <strong>DDS</strong>.
            <br />
            Los nodos se descubren entre sí automáticamente. Si un nodo muere, los demás siguen
            funcionando. Es mucho más robusto y tolerante a fallos.
          </TextBlock>
          <div class="architecture-box ros2">
            <div class="box-node">Nodo A</div>
            <div class="line-connect">DDS (Bus de Datos)</div>
            <div class="box-node">Nodo B</div>
            <div class="arrow-down">⬇</div>
            <div class="arrow-down">⬇</div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 2. ¿QUÉ ES DDS? -->
    <div class="section-group">
      <SectionTitle>2. El Motor Secreto: DDS</SectionTitle>
      <TextBlock>
        **DDS** no es un invento de ROS. Es un estándar industrial usado en **naves espaciales,
        presas hidroeléctricas y transacciones financieras**. ROS 2 simplemente se montó sobre
        hombros de gigantes.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="text-h6 text-accent">Descubrimiento Automático</div>
            <p>
              Cuando enciendes un nodo ROS 2, este grita a la red:
              <em>"¡Hola! Soy un sensor LIDAR y publico datos de distancia"</em>. <br /><br />
              Cualquier otro nodo interesado lo escucha y se conecta. No necesitas configurar IPs
              manualmente.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="text-h6 text-secondary">Abstracción (RMW)</div>
            <p>
              Tú escribes código ROS (Python/C++). Debajo, ROS traduce eso a DDS.
              <br /><br />
              Puedes cambiar la marca de DDS (FastDDS, CycloneDDS) sin cambiar una sola línea de tu
              código.
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. QoS: Calidad de Servicio -->
    <div class="section-group">
      <SectionTitle>3. QoS: Calidad de Servicio</SectionTitle>

      <AlertBlock type="info" title="El Superpoder de DDS">
        En ROS 1, todo era TCP (confiable pero lento). En ROS 2, gracias a DDS, puedes elegir cómo
        viajan tus datos.
      </AlertBlock>

      <TextBlock> Imagina dos escenarios diferentes en tu robot: </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="qos-card best-effort">
            <div class="text-weight-bold">Best Effort (Esfuerzo Máximo)</div>
            <div class="text-caption q-my-sm">Como streaming de video (UDP)</div>
            <p>
              "Lanzo los datos rápido. Si se pierde alguno, no me importa, ya viene el siguiente
              frame."
              <br />
              <strong>Uso:</strong> Cámaras, Sensores de alta velocidad.
            </p>
          </div>
        </template>
        <template #right>
          <div class="qos-card reliable">
            <div class="text-weight-bold">Reliable (Confiable)</div>
            <div class="text-caption q-my-sm">Como descargar un archivo (TCP)</div>
            <p>
              "El dato DEBE llegar. Si no llega, lo reenvío hasta que confirmes recepción."
              <br />
              <strong>Uso:</strong> Botón de emergencia, Configuración de misión.
            </p>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. DIAGNÓSTICO -->
    <div class="section-group">
      <SectionTitle>4. Verificando tu Middleware</SectionTitle>
      <TextBlock>
        Por defecto, ROS 2 (Jazzy/Humble) usa **Fast DDS** (de eProsima). Vamos a usar la
        herramienta <code>ros2 doctor</code> para ver qué tenemos bajo el capó.
      </TextBlock>

      <CodeBlock
        title="Reporte del Doctor"
        lang="bash"
        content="# Ver información general del middleware
ros2 doctor --report | grep middleware

# Salida esperada:
# middleware name    : rmw_fastrtps_cpp
# middleware version : ..."
        :copyable="true"
      />
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto Conceptual</SectionTitle>
      <TextBlock>
        Responde mentalmente a estas situaciones. ¿Qué política de QoS (Reliable o Best Effort)
        usarías?
      </TextBlock>

      <div class="q-pl-md">
        <ol class="text-grey-4">
          <li class="q-mb-sm">
            El robot envía una foto de alta resolución a la estación base cada 100ms.
            <strong class="text-accent">(R: Best Effort)</strong>
          </li>
          <li class="q-mb-sm">
            El operador envía el comando "¡DETENERSE AHORA!".
            <strong class="text-accent">(R: Reliable)</strong>
          </li>
          <li class="q-mb-sm">
            Lecturas de batería (se actualiza cada 1 segundo).
            <strong class="text-accent"
              >(R: Reliable - usualmente queremos saber el % exacto)</strong
            >
          </li>
        </ol>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

/* Diagramas de Arquitectura simplificados */
.architecture-box {
  background: var(--bg-surface-hover);
  padding: 1rem;
  border-radius: 8px;
  text-align: center;
  margin-top: 1rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 5px;
}
.box-node {
  background: var(--bg-surface-solid);
  padding: 4px 12px;
  border-radius: 4px;
  font-size: 0.8rem;
  border: 1px solid #475569;
}
.box-master {
  background: #f87171;
  color: black;
  font-weight: bold;
  padding: 8px 20px;
  border-radius: 50%;
  margin: 5px 0;
}
.line-connect {
  background: #4ade80;
  color: black;
  width: 100%;
  padding: 4px;
  font-size: 0.8rem;
  font-weight: bold;
}

/* Tarjetas Conceptuales */
.concept-card {
  background: var(--bg-surface-solid);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
}
.concept-card p {
  color: var(--text-secondary);
  font-size: 0.95rem;
}

/* Tarjetas QoS */
.qos-card {
  padding: 1rem;
  border-radius: 8px;
  height: 100%;
}
.qos-card.best-effort {
  background: rgba(251, 146, 60, 0.1);
  border-left: 3px solid #fb923c; /* Orange */
}
.qos-card.reliable {
  background: rgba(56, 189, 248, 0.1);
  border-left: 3px solid #38bdf8; /* Blue */
}
</style>
