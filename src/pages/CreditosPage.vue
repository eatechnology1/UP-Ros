<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO: LEGADO -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">Opción de Grado</div>
        <h1 class="hero-title">Proyecto <span class="text-primary">UP-Ros</span></h1>

        <TextBlock>
          UP-Ros surge como una síntesis entre la formación académica y la experiencia práctica. Es
          una solución educativa integral diseñada para abordar el aprendizaje de la robótica
          moderna desde una perspectiva sistémica, uniendo diseño de software, fundamentos de
          control y experiencia de usuario.
        </TextBlock>
      </div>
    </section>

    <!-- 2. EL AUTOR -->
    <div class="section-group self-stretch">
      <SectionTitle>Sobre el Autor</SectionTitle>

      <SplitBlock>
        <template #left>
          <div class="profile-header q-mb-md">
            <h2 class="text-h4 text-white q-my-none">Alexander Calderon Leal</h2>
            <div class="text-subtitle1 text-primary text-weight-bold">
              Ingeniero Mecatrónico | Robótica & Automatización
            </div>
            <div class="text-caption text-grey-5 q-mt-xs">
              <q-icon name="place" size="xs" /> Cúcuta, Norte de Santander, CO
            </div>
          </div>

          <TextBlock>
            Ingeniero Mecatrónico especializado en la integración coherente de mecánica, electrónica
            y software. Mi trayectoria abarca el diseño de sistemas mecatrónicos robustos, desde
            <strong>control de movimiento</strong> y <strong>sistemas embebidos</strong> hasta redes
            industriales. <br /><br />
            Mi enfoque central es la <strong>robótica móvil basada en ROS 2</strong>, donde
            desarrollo arquitecturas de software, simulación (Gazebo) y navegación autónoma. Combino
            el rigor técnico de la ingeniería con herramientas modernas como Python, MATLAB y
            frameworks web para crear soluciones escalables y reproducibles.
          </TextBlock>

          <!-- Botones Sociales -->
          <div class="row q-gutter-sm q-mt-lg">
            <q-btn
              color="primary"
              icon="code"
              label="GitHub"
              outline
              no-caps
              href="https://github.com/"
              target="_blank"
            />
            <q-btn
              color="blue-9"
              icon="work"
              label="LinkedIn"
              outline
              no-caps
              href="https://linkedin.com/"
              target="_blank"
            />
            <q-btn
              color="white"
              icon="mail"
              label="Contacto"
              flat
              no-caps
              href="mailto:alexander@example.com"
            />
          </div>
        </template>

        <template #right>
          <div class="philosophy-card">
            <q-icon name="auto_stories" size="4rem" color="accent" class="q-mb-md" />
            <h3 class="text-h6 text-white">Visión Académica</h3>
            <p class="text-grey-4 text-justify">
              "La correcta documentación, la estructuración del conocimiento y la visualización
              clara son claves para el aprendizaje efectivo en ingeniería. Mi compromiso es conectar
              la teoría con la práctica mediante plataformas educativas bien documentadas y
              alineadas con estándares profesionales."
            </p>
            <q-separator color="grey-8" class="q-my-md" />
            <div class="row justify-center q-gutter-sm">
              <q-chip dense color="dark" text-color="blue-3" icon="settings">Control</q-chip>
              <q-chip dense color="dark" text-color="green-3" icon="memory">Embebidos</q-chip>
              <q-chip dense color="dark" text-color="orange-3" icon="smart_toy">ROS 2</q-chip>
              <q-chip dense color="dark" text-color="purple-3" icon="school">Docencia</q-chip>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 3. STACK TECNOLÓGICO EXPANDIDO -->
    <div class="section-group self-stretch">
      <SectionTitle>Stack Tecnológico & Herramientas</SectionTitle>
      <TextBlock>
        La integración mecatrónica requiere dominio transversal. Este proyecto y mi perfil
        profesional se construyen sobre tres pilares tecnológicos:
      </TextBlock>

      <!-- GRUPO 1: ROBÓTICA -->
      <div class="tech-category q-mt-lg">
        <div class="category-title text-purple-3">🤖 Robótica & Simulación</div>
        <div class="tech-grid">
          <div v-for="tech in stackRobotics" :key="tech.name" class="tech-item">
            <q-icon :name="tech.icon" size="md" :color="tech.color" />
            <div class="tech-name">{{ tech.name }}</div>
            <div class="tech-role">{{ tech.role }}</div>
          </div>
        </div>
      </div>

      <!-- GRUPO 2: DESARROLLO WEB -->
      <div class="tech-category q-mt-lg">
        <div class="category-title text-blue-3">💻 Desarrollo Web & Apps</div>
        <div class="tech-grid">
          <div v-for="tech in stackWeb" :key="tech.name" class="tech-item">
            <q-icon :name="tech.icon" size="md" :color="tech.color" />
            <div class="tech-name">{{ tech.name }}</div>
            <div class="tech-role">{{ tech.role }}</div>
          </div>
        </div>
      </div>

      <!-- GRUPO 3: INGENIERÍA -->
      <div class="tech-category q-mt-lg">
        <div class="category-title text-orange-3">⚙️ Ingeniería & Herramientas</div>
        <div class="tech-grid">
          <div v-for="tech in stackEng" :key="tech.name" class="tech-item">
            <q-icon :name="tech.icon" size="md" :color="tech.color" />
            <div class="tech-name">{{ tech.name }}</div>
            <div class="tech-role">{{ tech.role }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. REFERENCIAS -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>Referencias & Recursos</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El contenido técnico de UP-Ros se fundamenta en la documentación oficial y estándares de
            la industria. Agradecimientos a las comunidades que hacen posible el ecosistema Open
            Source:
          </TextBlock>
          <ul class="ref-list">
            <li>
              <a href="https://docs.ros.org/en/jazzy/" target="_blank"
                >ROS 2 Documentation (Jazzy Jalisco)</a
              >
            </li>
            <li>
              <a href="https://gazebosim.org/docs" target="_blank">Gazebo Harmonic Simulator</a>
            </li>
            <li><a href="https://navigation.ros.org/" target="_blank">Nav2 Navigation Stack</a></li>
          </ul>
        </template>
        <template #right>
          <AlertBlock type="info" title="📄 Licencia Académica">
            Este proyecto se presenta como trabajo de grado. El material didáctico y el código
            fuente están disponibles para consulta y uso educativo, promoviendo la transferencia de
            conocimiento en la comunidad académica.
          </AlertBlock>
        </template>
      </SplitBlock>
    </div>

    <!-- FOOTER -->
    <div class="text-center text-grey-6 q-pb-xl text-caption">
      &copy; {{ new Date().getFullYear() }} Alexander Calderon Leal. Desarrollado en Colombia.
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';

// STACK ROBÓTICA
const stackRobotics = [
  { name: 'ROS 2 Jazzy', role: 'Middleware Distribuido', icon: 'smart_toy', color: 'purple-5' },
  { name: 'Gazebo', role: 'Simulación Física', icon: 'public', color: 'orange-5' },
  { name: 'Python', role: 'Scripting & Nodos', icon: 'terminal', color: 'yellow-8' },
  { name: 'Nav2', role: 'Navegación Autónoma', icon: 'explore', color: 'green-5' },
];

// STACK WEB
const stackWeb = [
  { name: 'Vue.js 3', role: 'Framework Reactivo', icon: 'layers', color: 'green-4' },
  { name: 'Quasar', role: 'UI/UX Material', icon: 'style', color: 'blue-5' },
  { name: 'TypeScript', role: 'Lógica Tipada', icon: 'code', color: 'blue-8' },
  { name: 'PWA', role: 'Progressive Web App', icon: 'devices', color: 'cyan-5' },
];

// STACK INGENIERÍA
const stackEng = [
  { name: 'MATLAB', role: 'Análisis de Control', icon: 'analytics', color: 'red-5' },
  { name: 'Embedded C', role: 'Microcontroladores', icon: 'memory', color: 'grey-5' },
  { name: 'Git', role: 'Control de Versiones', icon: 'source', color: 'orange-8' },
  { name: 'Linux', role: 'OS (Ubuntu)', icon: 'laptop', color: 'grey-4' },
];
</script>

<style scoped>
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1000px;
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at bottom, rgba(139, 92, 246, 0.15), transparent 60%),
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

/* PERFIL */
.philosophy-card {
  background: linear-gradient(135deg, rgba(30, 41, 59, 0.6), rgba(15, 23, 42, 0.9));
  padding: 32px;
  border-radius: 16px;
  border: 1px solid rgba(255, 255, 255, 0.05);
  text-align: center;
  height: 100%;
  display: flex;
  flex-direction: column;
  justify-content: center;
  box-shadow: 0 20px 40px rgba(0, 0, 0, 0.2);
}

/* CATEGORÍAS DE STACK */
.tech-category {
  margin-bottom: 2rem;
}

.category-title {
  font-size: 1.1rem;
  font-weight: 700;
  margin-bottom: 1rem;
  text-transform: uppercase;
  letter-spacing: 1px;
  border-left: 3px solid currentColor;
  padding-left: 10px;
}

.tech-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 16px;
}

.tech-item {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  padding: 16px;
  border-radius: 12px;
  text-align: center;
  transition: all 0.3s ease;
}

.tech-item:hover {
  background: rgba(30, 41, 59, 0.7);
  transform: translateY(-5px);
  border-color: rgba(56, 189, 248, 0.3);
}

.tech-name {
  color: #f1f5f9;
  font-weight: 700;
  margin-top: 8px;
  font-size: 1rem;
}

.tech-role {
  color: #94a3b8;
  font-size: 0.8rem;
}

/* REFERENCIAS */
.ref-list {
  color: #cbd5e1;
  line-height: 1.8;
  list-style-type: none;
  padding-left: 0;
}

.ref-list li::before {
  content: '🔗 ';
}

.ref-list a {
  color: #38bdf8;
  text-decoration: none;
  border-bottom: 1px dashed rgba(56, 189, 248, 0.3);
  transition: border-color 0.2s;
}

.ref-list a:hover {
  border-bottom-style: solid;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
