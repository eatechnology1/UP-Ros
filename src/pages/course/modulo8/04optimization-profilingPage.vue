<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      Si tu robot autónomo frena 500ms tarde, choca. La optimización no es un lujo, es seguridad.
      Aprenderemos a usar <strong>Zero-Copy</strong> para pasar gigabytes de datos sin copiar
      memoria y a tunear las políticas de <strong>QoS (Quality of Service)</strong>.
    </TextBlock>

    <!-- ZERO COPY EXPLAINED -->
    <div class="section-group">
      <SectionTitle>1. Zero-Copy: Loaned Messages</SectionTitle>
      <TextBlock>
        En ROS 1, enviar una imagen de cámara (1080p) implicaba serializar y copiar datos 2 o 3
        veces en RAM. En ROS 2, con <strong>Loaned Messages</strong>, el publicador "presta" un
        puntero de memoria al suscriptor. ¡Cero copias!
      </TextBlock>

      <div class="comparison-viz q-mt-md">
        <div class="method standard">
          <div class="method-title">Standard Publish (Slow)</div>
          <div class="mem-blocks">
            <div class="block">User App</div>
            <div class="copy-arrow">➜ Copy ➜</div>
            <div class="block">DDS Buffer</div>
            <div class="copy-arrow">➜ Copy ➜</div>
            <div class="block">Subscriber</div>
          </div>
          <div class="perf-metric bad">Latency: ~5ms (1080p)</div>
        </div>

        <div class="method zero">
          <div class="method-title">Zero-Copy (Loaned)</div>
          <div class="mem-shared">
            <div class="block-shared">Shared Memory (/dev/shm)</div>
            <div class="pointers">
              <span class="p-arrow">⬆ Ptr</span>
              <span class="p-arrow">⬆ Ptr</span>
            </div>
            <div class="actors">
              <span>Publisher</span>
              <span>Subscriber</span>
            </div>
          </div>
          <div class="perf-metric good">Latency: ~0.1ms (1080p) 🚀</div>
        </div>
      </div>

      <CodeBlock
        title="Zero-Copy Publisher (C++)"
        lang="cpp"
        content="// 1. Solicitar memoria al Middleware
auto loaned_msg = pub_->borrow_loaned_message();

// 2. Escribir datos directamente en la memoria prestada
loaned_msg.get().data = my_big_image_data;

// 3. Publicar (Solo se pasa el puntero)
pub_->publish(std::move(loaned_msg));"
        :copyable="true"
        class="q-mt-lg"
      />
    </div>

    <!-- QOS TUNER -->
    <div class="section-group">
      <SectionTitle>2. Quality of Service (QoS) Tuner</SectionTitle>
      <TextBlock>
        TCP garantiza entrega pero es lento (ROS 1). UDP es rápido pero pierde paquetes. ROS 2 te
        deja elegir con QoS Policies.
      </TextBlock>

      <div class="qos-lab q-mt-md">
        <div class="qos-scenario">
          <div class="sc-title">Scenario: Teleop (Joystick)</div>
          <div class="sc-desc">
            Si pierdo un paquete de "velocidad", no me importa, ya viene el siguiente. Quiero
            latencia mínima.
          </div>
          <div class="recommended-qos">
            <span class="policy">Reliability: BEST_EFFORT</span>
            <span class="policy">Durability: VOLATILE</span>
          </div>
        </div>

        <div class="qos-scenario critical">
          <div class="sc-title">Scenario: Emergency Stop</div>
          <div class="sc-desc">
            Este mensaje DEBE llegar sí o sí, aunque tardé un poco. Y si el nodo entra tarde, debe
            verlo.
          </div>
          <div class="recommended-qos">
            <span class="policy">Reliability: RELIABLE</span>
            <span class="policy">Durability: TRANSIENT_LOCAL</span>
          </div>
        </div>
      </div>
    </div>

    <!-- TRACING & PROFILING -->
    <div class="section-group">
      <SectionTitle>3. Forense Digital: Tracing con LTTng</SectionTitle>
      <TextBlock>
        ¿Por qué mi callback tarda tanto? <strong>ros2_tracing</strong> instrumenta el kernel para
        darte una línea de tiempo exacta de cada mensaje.
      </TextBlock>

      <div class="trace-viz">
        <div class="trace-line">
          <div class="event pub">rclcpp_publish</div>
          <div class="duration-bar" style="width: 20%"></div>
        </div>
        <div class="trace-line">
          <div class="event dds">dds_write</div>
          <div class="duration-bar" style="width: 10%"></div>
        </div>
        <div class="trace-line">
          <div class="event net">network_latency</div>
          <div class="duration-bar warn" style="width: 60%">???? (DDS Discovery Issues)</div>
        </div>
        <div class="trace-line">
          <div class="event sub">rclcpp_take</div>
          <div class="duration-bar" style="width: 10%"></div>
        </div>
      </div>

      <CodeBlock
        title="Trace Command"
        lang="bash"
        content="ros2 trace --session my_trace --list
# Genera un archivo CTF visualizable en Trace Compass"
        :copyable="true"
        class="q-mt-sm"
      />
    </div>

    <!-- DOCTORAL CHALLENGE -->
    <div class="section-group">
      <SectionTitle>4. Doctor''s Challenge: The Incompatible QoS</SectionTitle>

      <div class="challenge-card">
        <div class="challenge-title">🔇 El Silencio de los Inocentes</div>
        <div class="challenge-desc">
          Tienes un Publisher configurado como <strong>BEST_EFFORT</strong> (Lidar rápido). Tu
          Subscriber (Rviz) está configurado como <strong>RELIABLE</strong> (Default).
          <br />
          Resultado: No ves nada. ¿Por qué?
        </div>

        <div class="options-grid">
          <div class="option wrong">
            <div class="opt-head">A. Bug en Rviz</div>
            <div class="opt-body">No, Rviz funciona bien.</div>
          </div>
          <div class="option correct">
            <div class="opt-head">B. Incompatibilidad QoS (Rx > Tx)</div>
            <div class="opt-body">
              Exacto. Regla: <code>Reliability(Subscriber) <= Reliability(Publisher)</code>. No
              puedes pedir fiabilidad (Reliable) a alguien que no la ofrece (Best Effort).
            </div>
          </div>
          <div class="option wrong">
            <div class="opt-head">C. Firewall bloqueando UDP</div>
            <div class="opt-body">Si fuera firewall, ningún QoS funcionaría.</div>
          </div>
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Zero-Copy</code>
          <span>Paso de mensajes por referencia (Shared Memory).</span>
        </div>
        <div class="summary-item">
          <code>Transient Local</code>
          <span>QoS: Persistencia de mensajes para Late-Joiners (como 'Latched' en ROS 1).</span>
        </div>
        <div class="summary-item">
          <code>LTTng</code>
          <span>Linux Trace Toolkit: Profiling de bajo overhead.</span>
        </div>
        <div class="summary-item">
          <code>ros2 param dump</code>
          <span>Backup de configuraciones en YAML.</span>
        </div>
      </div>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

/* COMPARISON VIZ */
.comparison-viz {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 2rem;
  margin-bottom: 2rem;
}

.method {
  background: rgba(15, 23, 42, 0.8);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  align-items: center;
}

.method-title {
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.mem-blocks {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  width: 100%;
  align-items: center;
}

.block {
  background: #334155;
  color: #fff;
  padding: 0.5rem 1rem;
  border-radius: 4px;
  width: 80%;
  text-align: center;
}

.copy-arrow {
  color: #ef4444;
  font-size: 0.8rem;
  font-weight: 700;
}

.mem-shared {
  background: #1e293b;
  border: 2px dashed #22c55e;
  padding: 1rem;
  border-radius: 8px;
  width: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
}

.block-shared {
  background: rgba(34, 197, 94, 0.2);
  color: #4ade80;
  padding: 0.5rem;
  border-radius: 4px;
  margin-bottom: 1rem;
  font-weight: 700;
}

.pointers {
  display: flex;
  justify-content: space-around;
  width: 100%;
}
.p-arrow {
  color: #22c55e;
  font-family: monospace;
}

.actors {
  display: flex;
  justify-content: space-between;
  width: 100%;
  font-size: 0.8rem;
  color: #94a3b8;
  margin-top: 0.5rem;
}

.perf-metric {
  margin-top: 1rem;
  font-family: monospace;
  font-weight: 700;
  padding: 0.5rem 1rem;
  border-radius: 4px;
}
.perf-metric.bad {
  background: rgba(239, 68, 68, 0.2);
  color: #ef4444;
}
.perf-metric.good {
  background: rgba(34, 197, 94, 0.2);
  color: #22c55e;
}

/* QOS LAB */
.qos-lab {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1.5rem;
}

.qos-scenario {
  background: rgba(30, 41, 59, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.qos-scenario.critical {
  border-left: 5px solid #ef4444;
}

.sc-title {
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}
.sc-desc {
  font-size: 0.9rem;
  color: #cbd5e1;
  margin-bottom: 1rem;
}

.recommended-qos {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.policy {
  background: #0f172a;
  color: #a5f3fc;
  font-family: monospace;
  font-size: 0.8rem;
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
}

/* TRACE VIZ */
.trace-viz {
  background: #0f172a;
  padding: 1.5rem;
  border-radius: 8px;
  border: 1px solid #334155;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.trace-line {
  display: flex;
  align-items: center;
  gap: 1rem;
}

.event {
  width: 120px;
  font-family: monospace;
  font-size: 0.8rem;
  color: #94a3b8;
  text-align: right;
}

.duration-bar {
  height: 20px;
  background: #3b82f6;
  border-radius: 4px;
  font-size: 0.7rem;
  color: rgba(255, 255, 255, 0.8);
  display: flex;
  align-items: center;
  padding-left: 5px;
  white-space: nowrap;
  overflow: hidden;
}

.duration-bar.warn {
  background: #ef4444;
}

/* CHALLENGE */
.challenge-card {
  background: rgba(30, 41, 59, 0.8);
  border-radius: 16px;
  padding: 2rem;
  border-left: 5px solid #8b5cf6;
}

.challenge-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #c4b5fd;
  margin-bottom: 0.5rem;
}
.challenge-desc {
  color: #cbd5e1;
  margin-bottom: 1.5rem;
}

.options-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1rem;
}

.option {
  background: rgba(0, 0, 0, 0.3);
  padding: 1rem;
  border-radius: 8px;
  cursor: pointer;
}

.option.correct:active {
  border: 1px solid #22c55e;
}
.option.wrong:active {
  border: 1px solid #ef4444;
}

.opt-head {
  font-weight: 700;
  color: #cbd5e1;
  margin-bottom: 0.5rem;
}
.opt-body {
  font-size: 0.85rem;
  color: #94a3b8;
}

/* SUMMARY */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: rgba(15, 23, 42, 0.6);
  padding: 1rem;
  border-radius: 8px;
  display: flex;
  flex-direction: column;
}

.summary-item code {
  color: #22c55e;
  font-family: monospace;
}
.summary-item span {
  font-size: 0.85rem;
  color: #94a3b8;
}

@media (max-width: 1024px) {
  .options-grid,
  .comparison-viz,
  .qos-lab {
    grid-template-columns: 1fr;
  }
}
</style>
