<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      ROS 1 no tenía seguridad: cualquiera en la red podía mover tu robot. <strong>SROS 2</strong>
      utiliza la especificación de Seguridad de DDS para implementar Autenticación, Cifrado y
      Control de Acceso.
      <br /><br />
      Es el mismo estándar usado en bancos y defensa. Sin las llaves correctas, tu robot es un
      ladrillo.
    </TextBlock>

    <!-- DIDACTIC VIZ: SECURITY LAYERS -->
    <div class="section-group">
      <SectionTitle>1. Anatomía de la Seguridad (DDS Security)</SectionTitle>

      <div class="security-arch-viz q-mt-md">
        <div class="shield-layer auth">
          <q-icon name="badge" size="md" />
          <div class="layer-title">Authentication (PKI)</div>
          <div class="layer-desc">¿Eres quien dices ser? (Certificados x.509)</div>
        </div>

        <div class="shield-arrow">⬇ Si OK</div>

        <div class="shield-layer access">
          <q-icon name="policy" size="md" />
          <div class="layer-title">Access Control (Governance)</div>
          <div class="layer-desc">¿Tienes permiso para publicar en /cmd_vel?</div>
        </div>

        <div class="shield-arrow">⬇ Si OK</div>

        <div class="shield-layer crypt">
          <q-icon name="vpn_key" size="md" />
          <div class="layer-title">Encryption (AES-GCM)</div>
          <div class="layer-desc">Datos ilegibles para espías (WireShark).</div>
        </div>
      </div>
    </div>

    <!-- SROS2 CLI -->
    <div class="section-group">
      <SectionTitle>2. Gestión de Llaves: SROS2 CLI</SectionTitle>
      <TextBlock>
        No generamos certificados con OpenSSL manualmente. Usamos la herramienta <code>s2</code>
        (ros2 security).
      </TextBlock>

      <div class="terminal-simulation">
        <div class="term-line">
          <span class="prompt">$</span> ros2 security create_keystore ~/robot_keystore
        </div>
        <div class="term-line output">Created keystore at: /home/user/robot_keystore</div>

        <div class="term-line">
          <span class="prompt">$</span> ros2 security create_key ~/robot_keystore /my_node
        </div>
        <div class="term-line output">Created key for context: /my_node</div>

        <div class="term-line"><span class="prompt">$</span> export ROS_SECURITY_ENABLE=true</div>
        <div class="term-line">
          <span class="prompt">$</span> export ROS_SECURITY_STRATEGY=Enforce
        </div>
        <div class="term-desc">
          Activación de modo estricto. Si no hay llave, el nodo muere al inicio.
        </div>
      </div>
    </div>

    <!-- PERMISSIONS XML -->
    <div class="section-group">
      <SectionTitle>3. Políticas de Acceso (Governance)</SectionTitle>
      <TextBlock>
        El archivo <code>permissions.xml</code> define granularmente qué puede hacer cada nodo. Es
        firmado criptográficamente.
      </TextBlock>

      <CodeBlock
        title="permissions.xml"
        lang="xml"
        content='<permissions>
  <grant name="/camera_node_permissions">
    <subject_name>CN=/camera_node</subject_name>
    <validity>
      <not_before>2023-01-01T00:00:00</not_before>
      <not_after>2033-01-01T00:00:00</not_after>
    </validity>
    <allow_rule>
      <!-- Solo puede publicar imágenes, NO cmd_vel -->
      <publish>
        <topic>image_raw</topic>
        <topic>camera_info</topic>
      </publish>
    </allow_rule>
    <default>DENY</default>
  </grant>
</permissions>'
        :copyable="true"
      />
    </div>

    <!-- DOCTORAL CHALLENGE -->
    <div class="section-group">
      <SectionTitle>4. Doctor''s Challenge: The Rogue Node</SectionTitle>

      <div class="challenge-card">
        <div class="challenge-title">🕵️ Intrusión Detectada</div>
        <div class="challenge-desc">
          En una red segura, un atacante conecta su laptop e intenta escuchar <code>/map</code>.
          <br />
          ¿Qué ve en Wireshark?
        </div>

        <div class="options-grid">
          <div class="option wrong">
            <div class="opt-head">A. El mapa en texto plano</div>
            <div class="opt-body">Imposible. El cifrado AES-256 protege el payload.</div>
          </div>
          <div class="option correct">
            <div class="opt-head">B. Basura ininteligible (UDP)</div>
            <div class="opt-body">
              Exacto. Ve paquetes RTPS, pero el contenido es ruido aleatorio cifrado. Ni siquiera
              sabe qué tópicos existen (Discovery cifrado).
            </div>
          </div>
          <div class="option wrong">
            <div class="opt-head">C. Nada (Silencio total)</div>
            <div class="opt-body">Ve tráfico UDP fluir, pero no puede interpretarlo.</div>
          </div>
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Keystore</code>
          <span>Carpeta segura con la CA (Certificate Authority) y llaves privadas.</span>
        </div>
        <div class="summary-item">
          <code>Governance.xml</code>
          <span>Reglas globales (¿Debo cifrar todo o solo algunos tópicos?).</span>
        </div>
        <div class="summary-item">
          <code>Permissions.xml</code>
          <span>Reglas por nodo (¿Quién puede publicar qué?).</span>
        </div>
        <div class="summary-item">
          <code>Enforce</code>
          <span>Estrategia que prohíbe nodos sin certs.</span>
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

/* SECURITY VIZ */
.security-arch-viz {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
  background: rgba(15, 23, 42, 0.8);
  padding: 2rem;
  border-radius: 12px;
}

.shield-layer {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 1rem 2rem;
  border-radius: 12px;
  width: 100%;
  max-width: 400px;
  text-align: center;
  border: 1px solid rgba(255, 255, 255, 0.1);
}

.shield-layer.auth {
  background: rgba(59, 130, 246, 0.2);
  border-color: #3b82f6;
  color: #93c5fd;
}
.shield-layer.access {
  background: rgba(234, 179, 8, 0.2);
  border-color: #eab308;
  color: #fde047;
}
.shield-layer.crypt {
  background: rgba(239, 68, 68, 0.2);
  border-color: #ef4444;
  color: #fca5a5;
}

.layer-title {
  font-weight: 700;
  margin-top: 0.5rem;
  font-size: 1.1rem;
}
.layer-desc {
  font-size: 0.85rem;
  opacity: 0.8;
  margin-top: 5px;
}

.shield-arrow {
  font-family: monospace;
  color: #64748b;
  font-weight: 700;
}

/* TERMINAL SIM */
.terminal-simulation {
  background: #0f172a;
  border-radius: 8px;
  padding: 1.5rem;
  font-family: 'Fira Code', monospace;
  border: 1px solid #334155;
  margin-top: 1rem;
}

.term-line {
  margin-bottom: 0.5rem;
  color: #e2e8f0;
  font-size: 0.9rem;
}
.prompt {
  color: #22c55e;
  margin-right: 0.5rem;
}
.output {
  color: #94a3b8;
  font-style: italic;
}
.term-desc {
  margin-top: 1rem;
  padding-top: 1rem;
  border-top: 1px solid #334155;
  color: #fbbf24;
  font-size: 0.85rem;
}

/* CHALLENGE */
.challenge-card {
  background: rgba(30, 41, 59, 0.8);
  border-radius: 16px;
  padding: 2rem;
  border-left: 5px solid #dc2626;
}

.challenge-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #f87171;
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
  color: #f87171;
  font-family: monospace;
}
.summary-item span {
  font-size: 0.85rem;
  color: #94a3b8;
}

@media (max-width: 1024px) {
  .options-grid {
    grid-template-columns: 1fr;
  }
}
</style>
