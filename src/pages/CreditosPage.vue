<template>
  <q-page class="credits-page">
    <div class="credits-container">
      <!-- ========================================
           1. HERO SECTION - PROYECTO UP-ROS
           ======================================== -->
      <section class="hero-section" data-aos="fade-down">
        <div class="hero-content">
          <div class="text-overline text-accent text-weight-bold q-mb-sm">Opción de Grado</div>
          <h1 class="hero-title">Proyecto <span class="text-gradient">UP-Ros Academy</span></h1>

          <p class="hero-description">
            <strong>UP-Ros Academy</strong> representa la convergencia entre rigor académico y
            excelencia técnica en la enseñanza de sistemas robóticos autónomos. Este proyecto
            constituye una contribución al ecosistema educativo de ROS 2, diseñado bajo principios
            de arquitectura de software modular, experiencia de usuario centrada en el aprendizaje
            progresivo y estándares de calidad doctoral.
          </p>
        </div>
      </section>

      <!-- ========================================
           2. EQUIPO DE DESARROLLO (QCard Premium)
           ======================================== -->
      <section class="team-section">
        <h2 class="section-title" data-aos="fade-right">
          <q-icon name="groups" size="32px" class="q-mr-sm" />
          Equipo de Desarrollo
        </h2>

        <div class="team-grid">
          <!-- AUTOR PRINCIPAL -->
          <q-card class="profile-card" data-aos="fade-up" data-aos-delay="100">
            <q-card-section class="profile-header">
              <div class="avatar-container">
                <img src="~assets/images/alex.png" alt="Alexander Calderon" class="avatar-img" />
              </div>
              <h3 class="profile-name">Alexander Calderón Leal</h3>
              <div class="profile-role primary-role">Desarrollador Principal & Arquitecto</div>
              <div class="profile-institution">Universidad de Pamplona</div>
            </q-card-section>

            <q-card-section class="profile-body">
              <p class="profile-bio">
                Ingeniero Mecatrónico especializado en sistemas embebidos, IoT, robótica aplicada y
                soluciones de automatización industrial. Experiencia en diseño electrónico, redes
                industriales, visión térmica e inteligencia aplicada a sistemas autónomos. Creador
                de la arquitectura UP-Ros y desarrollador de plataformas educativas interactivas
                para ROS 2.
              </p>
            </q-card-section>

            <q-card-actions class="profile-actions">
              <q-btn
                flat
                round
                dense
                color="green-4"
                :icon="remMailSendFill"
                href="mailto:edwin.calderon@unipamplona.edu.co"
                target="_blank"
              >
                <q-tooltip>Correo Institucional</q-tooltip>
              </q-btn>
              <q-btn
                flat
                round
                dense
                color="blue-4"
                :icon="remTelegramFill"
                href="https://t.me/ea_technology"
                target="_blank"
              >
                <q-tooltip>Telegram</q-tooltip>
              </q-btn>
              <q-btn
                flat
                round
                dense
                color="red-5"
                :icon="remYoutubeFill"
                href="https://www.youtube.com/@ea.technology"
                target="_blank"
              >
                <q-tooltip>YouTube</q-tooltip>
              </q-btn>
              <q-btn
                flat
                round
                dense
                color="grey-5"
                :icon="remGithubFill"
                href="https://github.com/eatechnology1"
                target="_blank"
              >
                <q-tooltip>GitHub</q-tooltip>
              </q-btn>
            </q-card-actions>
          </q-card>

          <!-- DIRECTOR DE PROYECTO -->
          <q-card class="profile-card" data-aos="fade-up" data-aos-delay="200">
            <q-card-section class="profile-header">
              <div class="avatar-container">
                <img src="~assets/images/alex.png" alt="Oscar Duque" class="avatar-img" />
              </div>
              <h3 class="profile-name">BSc. Ing. Oscar Manuel Duque Suárez</h3>
              <div class="profile-role director-role">Director de Proyecto</div>
              <div class="profile-institution">Docente - Facultad de Ingenierías</div>
            </q-card-section>

            <q-card-section class="profile-body">
              <p class="profile-bio">
                Ingeniero Mecatrónico (2007), Especialista en Mecatrónica Industrial y MSc. en
                Controles Industriales. Docente del Departamento de Ingeniería Mecatrónica desde
                2009, con estudios doctorales en Proyectos. Miembro del Grupo de Investigación en
                Automatización y Control (A&C). Líneas de investigación: sistemas de control,
                robótica e inteligencia artificial aplicada.
              </p>
            </q-card-section>

            <q-card-actions class="profile-actions">
              <q-btn
                flat
                round
                dense
                color="orange-4"
                :icon="remBuilding2Fill"
                href="https://scienti.minciencias.gov.co/cvlac/visualizador/generarCurriculoCv.do?cod_rh=0000930113"
                target="_blank"
              >
                <q-tooltip>CvLAC / Perfil Académico</q-tooltip>
              </q-btn>
              <q-btn
                flat
                round
                dense
                color="green-4"
                :icon="remMailSendFill"
                href="mailto:omduque@unipamplona.edu.co"
                target="_blank"
              >
                <q-tooltip>Correo Institucional</q-tooltip>
              </q-btn>
              <q-btn
                flat
                round
                dense
                color="red-5"
                :icon="remYoutubeFill"
                href="https://www.youtube.com/@BRANKS-ROBOTICACOLABORATIVA"
                target="_blank"
              >
                <q-tooltip>BRANKS - Robótica Colaborativa</q-tooltip>
              </q-btn>
            </q-card-actions>
          </q-card>
        </div>
      </section>

      <!-- ========================================
           3. STACK TECNOLÓGICO EXPANDIDO
           ======================================== -->
      <section class="tech-section">
        <h2 class="section-title" data-aos="fade-right">
          <q-icon name="code" size="32px" class="q-mr-sm" />
          Stack Tecnológico
        </h2>

        <!-- CORE ROBOTICS -->
        <div class="tech-category" data-aos="fade-up">
          <h3 class="category-title">
            <q-icon name="smart_toy" class="q-mr-sm" />
            Core Robotics (ROS 2 Ecosystem)
          </h3>
          <div class="tech-grid">
            <div v-for="tech in stackRobotics" :key="tech.name" class="tech-item">
              <q-icon :name="tech.icon" size="40px" :color="tech.color" />
              <div class="tech-name">{{ tech.name }}</div>
              <div class="tech-role">{{ tech.role }}</div>
            </div>
          </div>
        </div>

        <!-- FRONTEND & UX -->
        <div class="tech-category" data-aos="fade-up" data-aos-delay="100">
          <h3 class="category-title">
            <q-icon name="web" class="q-mr-sm" />
            Frontend & User Experience
          </h3>
          <div class="tech-grid">
            <div v-for="tech in stackFrontend" :key="tech.name" class="tech-item">
              <q-icon :name="tech.icon" size="40px" :color="tech.color" />
              <div class="tech-name">{{ tech.name }}</div>
              <div class="tech-role">{{ tech.role }}</div>
            </div>
          </div>
        </div>

        <!-- DEVELOPMENT & DEVOPS -->
        <div class="tech-category" data-aos="fade-up" data-aos-delay="200">
          <h3 class="category-title">
            <q-icon name="build_circle" class="q-mr-sm" />
            Development & DevOps
          </h3>
          <div class="tech-grid">
            <div v-for="tech in stackDevOps" :key="tech.name" class="tech-item">
              <q-icon :name="tech.icon" size="40px" :color="tech.color" />
              <div class="tech-name">{{ tech.name }}</div>
              <div class="tech-role">{{ tech.role }}</div>
            </div>
          </div>
        </div>
      </section>

      <!-- ========================================
           5. RESPALDO INSTITUCIONAL
           ======================================== -->
      <section class="institutional-section">
        <h2 class="section-title" data-aos="fade-right">
          <q-icon name="school" size="32px" class="q-mr-sm" />
          Sello de Calidad Académica
        </h2>

        <div class="institutional-container" data-aos="fade-up">
          <!-- ENCABEZADO -->
          <div class="institutional-header">
            <h3 class="university-name">Universidad de Pamplona</h3>
            <div class="university-year">FUNDADA EN 1960</div>
            <p class="university-quote">
              "Formando nuevas generaciones con sello de excelencia comprometidos<br />
              con la transformación social de las regiones y un país en paz"
            </p>
          </div>

          <!-- AGRADECIMIENTOS -->
          <div class="acknowledgments">
            <h4 class="acknowledgments-title">Agradecimientos</h4>
            <p class="acknowledgments-text">
              Este trabajo es el resultado de la formación integral recibida en la
              <strong>Universidad de Pamplona</strong>, institución con Acreditación Institucional
              de Alta Calidad otorgada por el Ministerio de Educación Nacional de Colombia.
            </p>
            <p class="acknowledgments-text">
              Agradecimiento especial al <strong>Programa de Ingeniería Mecatrónica</strong> por su
              compromiso con la excelencia académica, la investigación aplicada en robótica y
              control, y la proyección internacional de sus egresados.
            </p>
            <p class="acknowledgments-text">
              Al <strong>Grupo de Investigación en Automatización y Control (A&C)</strong>, por su
              liderazgo en el desarrollo de soluciones tecnológicas que impactan la región y el
              país.
            </p>
          </div>

          <!-- BADGES INSTITUCIONALES -->
          <div class="badges-grid">
            <div class="badge-card" data-aos="flip-left" data-aos-delay="100">
              <div class="logo-box">
                <img src="~assets/images/escudounipamplona.png" alt="Escudo Unipamplona" />
              </div>
              <div class="badge-info">
                <div class="badge-title">Alma Mater</div>
                <div class="badge-desc">Tradición y Excelencia Académica</div>
              </div>
            </div>

            <div class="badge-card" data-aos="flip-left" data-aos-delay="200">
              <div class="logo-box">
                <img src="~assets/images/Mecatronica.png" alt="Ingeniería Mecatrónica" />
              </div>
              <div class="badge-info">
                <div class="badge-title">Ingeniería Mecatrónica</div>
                <div class="badge-desc">Innovación y Desarrollo Tecnológico</div>
              </div>
            </div>

            <div class="badge-card" data-aos="flip-left" data-aos-delay="300">
              <div class="logo-box">
                <img src="~assets/images/logo_acreditacion_png.png" alt="CNA Acreditación" />
              </div>
              <div class="badge-info">
                <div class="badge-title">Alta Calidad</div>
                <div class="badge-desc">Reconocimiento del Ministerio de Educación</div>
              </div>
            </div>

            <div class="badge-card" data-aos="flip-left" data-aos-delay="400">
              <div class="logo-box">
                <img src="~assets/images/logo-equaa.png" alt="EQUAA Internacional" />
              </div>
              <div class="badge-info">
                <div class="badge-title">Certificación Global</div>
                <div class="badge-desc">Estándares Internacionales EQUAA</div>
              </div>
            </div>
          </div>
        </div>
      </section>

      <!-- FOOTER -->
      <div class="credits-footer">
        © {{ new Date().getFullYear() }} Alexander Calderón Leal. Desarrollado en Colombia con
        <q-icon name="favorite" color="red-5" size="16px" /> y ROS 2.
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import { onMounted } from 'vue';
import AOS from 'aos';
import 'aos/dist/aos.css';
import {
  remGithubFill,
  remMailSendFill,
  remTelegramFill,
  remYoutubeFill,
  remBuilding2Fill,
} from 'quasar-extras-svg-icons/remix-icons';

// Inicializar AOS (Animate On Scroll)
onMounted(() => {
  AOS.init({
    duration: 800,
    easing: 'ease-out-cubic',
    once: true,
    offset: 50,
  });
});

// STACK ROBÓTICA (ROS 2 Ecosystem)
const stackRobotics = [
  { name: 'ROS 2 Jazzy', role: 'Middleware DDS', icon: 'smart_toy', color: 'purple-5' },
  { name: 'Nav2', role: 'Navegación Autónoma', icon: 'explore', color: 'green-5' },
  { name: 'SLAM Toolbox', role: 'Mapeo y Localización', icon: 'map', color: 'blue-5' },
  { name: 'Gazebo Harmonic', role: 'Simulación Física', icon: 'public', color: 'orange-5' },
  { name: 'RViz2', role: 'Visualización 3D', icon: 'visibility', color: 'cyan-5' },
];

// STACK FRONTEND
const stackFrontend = [
  { name: 'Vue 3', role: 'Composition API', icon: 'layers', color: 'green-6' },
  { name: 'Quasar Framework', role: 'UI Components', icon: 'widgets', color: 'blue-6' },
  { name: 'TypeScript', role: 'Type Safety', icon: 'code', color: 'blue-7' },
  { name: 'Vite', role: 'Build Tool', icon: 'bolt', color: 'purple-6' },
];

// STACK DEVOPS
const stackDevOps = [
  { name: 'Git & GitHub', role: 'Control de Versiones', icon: 'source', color: 'grey-7' },
  { name: 'Docker', role: 'Containerización', icon: 'inventory_2', color: 'blue-8' },
  { name: 'VS Code', role: 'IDE Principal', icon: 'terminal', color: 'blue-5' },
];
</script>

<style scoped>
/* ========================================
   BASE STYLES
   ======================================== */
.credits-page {
  background: linear-gradient(180deg, #0f172a 0%, #1e293b 100%);
  min-height: 100vh;
  padding: 2rem 1rem;
}

.credits-container {
  max-width: 1200px;
  margin: 0 auto;
}

/* ========================================
   HERO SECTION
   ======================================== */
.hero-section {
  background:
    radial-gradient(circle at top, rgba(139, 92, 246, 0.15), transparent 70%), rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 24px;
  padding: 4rem 2rem;
  text-align: center;
  margin-bottom: 4rem;
  backdrop-filter: blur(10px);
}

.hero-title {
  font-size: 3.5rem;
  font-weight: 900;
  margin: 0 0 2rem 0;
  line-height: 1.1;
  color: #f8fafc;
}

.text-gradient {
  background: linear-gradient(135deg, #38bdf8 0%, #818cf8 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.hero-description {
  font-size: 1.1rem;
  line-height: 1.8;
  color: #cbd5e1;
  max-width: 900px;
  margin: 0 auto;
  text-align: justify;
}

/* ========================================
   SECTION TITLES
   ======================================== */
.section-title {
  font-size: 2rem;
  font-weight: 700;
  color: #f8fafc;
  margin-bottom: 2rem;
  display: flex;
  align-items: center;
  border-bottom: 2px solid rgba(56, 189, 248, 0.3);
  padding-bottom: 0.5rem;
}

/* ========================================
   TEAM SECTION (QCard Premium)
   ======================================== */
.team-section {
  margin-bottom: 4rem;
}

.team-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
  gap: 2rem;
}

.profile-card {
  background: rgba(30, 41, 59, 0.6);
  backdrop-filter: blur(20px);
  border: 1px solid rgba(148, 163, 184, 0.15);
  border-radius: 20px;
  transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
  overflow: hidden;
}

.profile-card:hover {
  transform: translateY(-8px) rotateX(2deg);
  border-color: rgba(56, 189, 248, 0.5);
  box-shadow:
    0 20px 40px rgba(56, 189, 248, 0.2),
    0 0 0 1px rgba(56, 189, 248, 0.3);
}

.profile-header {
  text-align: center;
  padding: 2rem 1.5rem 1rem;
  background: linear-gradient(180deg, rgba(56, 189, 248, 0.05), transparent);
}

.avatar-container {
  width: 140px;
  height: 140px;
  margin: 0 auto 1.5rem;
  padding: 4px;
  background: linear-gradient(135deg, #38bdf8, #818cf8, #c084fc);
  border-radius: 50%;
  box-shadow: 0 8px 24px rgba(56, 189, 248, 0.3);
  animation: pulse-ring 3s ease-in-out infinite;
}

@keyframes pulse-ring {
  0%,
  100% {
    box-shadow: 0 8px 24px rgba(56, 189, 248, 0.3);
  }
  50% {
    box-shadow: 0 8px 32px rgba(56, 189, 248, 0.5);
  }
}

.avatar-img {
  width: 100%;
  height: 100%;
  object-fit: cover;
  border-radius: 50%;
  border: 4px solid #0f172a;
}

.profile-name {
  font-size: 1.4rem;
  font-weight: 700;
  color: #f8fafc;
  margin: 0 0 0.5rem 0;
}

.profile-role {
  font-size: 1rem;
  font-weight: 600;
  margin-bottom: 0.25rem;
}

.primary-role {
  color: #38bdf8;
}

.director-role {
  color: #818cf8;
}

.profile-institution {
  font-size: 0.9rem;
  color: #94a3b8;
}

.profile-body {
  padding: 1.5rem;
}

.profile-bio {
  font-size: 0.95rem;
  line-height: 1.7;
  color: #cbd5e1;
  text-align: justify;
  margin: 0;
}

.profile-actions {
  justify-content: center;
  padding: 1rem 1.5rem 1.5rem;
  gap: 0.5rem;
}

.profile-actions .q-btn {
  transition: all 0.3s ease;
}

.profile-actions .q-btn:hover {
  transform: scale(1.15);
  animation: pulse 0.6s ease-in-out;
}

@keyframes pulse {
  0%,
  100% {
    transform: scale(1.15);
  }
  50% {
    transform: scale(1.25);
  }
}

/* ========================================
   TECH STACK SECTION
   ======================================== */
.tech-section {
  margin-bottom: 4rem;
}

.tech-category {
  margin-bottom: 2.5rem;
}

.category-title {
  font-size: 1.3rem;
  font-weight: 600;
  color: #e2e8f0;
  margin-bottom: 1.5rem;
  display: flex;
  align-items: center;
}

.tech-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(160px, 1fr));
  gap: 1.5rem;
}

.tech-item {
  background: rgba(30, 41, 59, 0.5);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  padding: 1.5rem 1rem;
  text-align: center;
  transition: all 0.3s ease;
}

.tech-item:hover {
  transform: translateY(-4px) scale(1.05);
  background: rgba(30, 41, 59, 0.7);
  border-color: rgba(148, 163, 184, 0.3);
  box-shadow: 0 8px 16px rgba(0, 0, 0, 0.2);
}

.tech-name {
  color: #f1f5f9;
  font-weight: 700;
  margin-top: 0.75rem;
  font-size: 1rem;
}

.tech-role {
  color: #94a3b8;
  font-size: 0.85rem;
  margin-top: 0.25rem;
}

/* ========================================
   AI SECTION (NUEVA)
   ======================================== */
.ai-section {
  margin-bottom: 4rem;
}

.ai-card {
  background: linear-gradient(135deg, rgba(139, 92, 246, 0.15), rgba(56, 189, 248, 0.15));
  backdrop-filter: blur(20px);
  border: 2px solid rgba(139, 92, 246, 0.3);
  border-radius: 24px;
  overflow: hidden;
}

.ai-header {
  text-align: center;
  padding: 3rem 2rem 2rem;
  background: linear-gradient(180deg, rgba(139, 92, 246, 0.1), transparent);
}

.ai-icon-container {
  margin-bottom: 1rem;
}

.ai-icon {
  color: #c084fc;
  animation: rotate-glow 4s linear infinite;
}

@keyframes rotate-glow {
  0%,
  100% {
    filter: drop-shadow(0 0 8px rgba(192, 132, 252, 0.6));
  }
  50% {
    filter: drop-shadow(0 0 16px rgba(192, 132, 252, 0.9));
  }
}

.ai-title {
  font-size: 1.8rem;
  font-weight: 700;
  color: #f8fafc;
  margin: 0 0 1rem 0;
}

.ai-badge {
  font-size: 0.75rem;
  padding: 0.5rem 1rem;
  letter-spacing: 0.05em;
}

.ai-body {
  padding: 2rem;
}

.ai-description {
  font-size: 1.1rem;
  line-height: 1.8;
  color: #e2e8f0;
  text-align: center;
  margin-bottom: 2rem;
}

.ai-contributions {
  background: rgba(15, 23, 42, 0.4);
  border-radius: 16px;
  padding: 1.5rem;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.contributions-title {
  font-size: 1.1rem;
  font-weight: 600;
  color: #c084fc;
  margin: 0 0 1rem 0;
}

.contributions-list {
  list-style: none;
  padding: 0;
  margin: 0;
}

.contributions-list li {
  padding: 0.5rem 0 0.5rem 2rem;
  color: #cbd5e1;
  position: relative;
}

.contributions-list li::before {
  content: '✓';
  position: absolute;
  left: 0;
  color: #38bdf8;
  font-weight: 700;
  font-size: 1.2rem;
}

/* ========================================
   INSTITUTIONAL SECTION
   ======================================== */
.institutional-section {
  margin-bottom: 4rem;
}

.institutional-container {
  background: linear-gradient(180deg, rgba(30, 41, 59, 0.4), rgba(15, 23, 42, 0.6));
  border: 1px solid rgba(255, 255, 255, 0.08);
  border-radius: 24px;
  padding: 3rem 2rem;
  position: relative;
  overflow: hidden;
}

.institutional-container::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  height: 4px;
  background: linear-gradient(90deg, #aa1c3a, #d90429, #aa1c3a);
}

.institutional-header {
  text-align: center;
  margin-bottom: 3rem;
}

.university-name {
  font-size: 2.5rem;
  font-weight: 700;
  color: #f8fafc;
  margin: 0 0 0.5rem 0;
}

.university-year {
  font-size: 0.9rem;
  color: #94a3b8;
  letter-spacing: 0.15em;
  margin-bottom: 1.5rem;
}

.university-quote {
  font-style: italic;
  font-family: 'Times New Roman', serif;
  color: #e2e8f0;
  font-size: 1.2rem;
  line-height: 1.6;
  margin: 0;
}

.acknowledgments {
  background: rgba(255, 255, 255, 0.02);
  border-radius: 16px;
  padding: 2rem;
  margin-bottom: 3rem;
  border: 1px solid rgba(255, 255, 255, 0.05);
}

.acknowledgments-title {
  font-size: 1.5rem;
  font-weight: 600;
  color: #38bdf8;
  margin: 0 0 1.5rem 0;
  text-align: center;
}

.acknowledgments-text {
  font-size: 1rem;
  line-height: 1.8;
  color: #cbd5e1;
  text-align: justify;
  margin-bottom: 1rem;
}

.acknowledgments-text:last-child {
  margin-bottom: 0;
}

.badges-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
  gap: 2rem;
}

.badge-card {
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  background: rgba(255, 255, 255, 0.03);
  border-radius: 20px;
  padding: 1.5rem;
  border: 1px solid transparent;
  transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
}

.badge-card:hover {
  transform: translateY(-6px);
  background: rgba(255, 255, 255, 0.06);
  border-color: rgba(56, 189, 248, 0.3);
  box-shadow: 0 12px 24px rgba(0, 0, 0, 0.3);
}

.logo-box {
  background: linear-gradient(135deg, #ffffff 0%, #f8f9fa 100%);
  width: 100%;
  height: 110px;
  border-radius: 16px;
  display: flex;
  justify-content: center;
  align-items: center;
  padding: 1rem;
  margin-bottom: 1.25rem;
  border: 2px solid rgba(56, 189, 248, 0.1);
  box-shadow:
    0 4px 12px rgba(0, 0, 0, 0.1),
    inset 0 1px 0 rgba(255, 255, 255, 0.8);
  transition: all 0.3s ease;
}

.badge-card:hover .logo-box {
  transform: scale(1.05);
  border-color: rgba(56, 189, 248, 0.4);
  box-shadow:
    0 8px 24px rgba(56, 189, 248, 0.2),
    inset 0 1px 0 rgba(255, 255, 255, 1);
}

.logo-box img {
  max-width: 100%;
  max-height: 100%;
  object-fit: contain;
  filter: saturate(0.95);
  transition: filter 0.3s ease;
}

.badge-card:hover .logo-box img {
  filter: saturate(1.1);
}

.badge-title {
  color: #f8fafc;
  font-weight: 700;
  font-size: 1.1rem;
  margin-bottom: 0.5rem;
}

.badge-desc {
  color: #94a3b8;
  font-size: 0.9rem;
  line-height: 1.5;
}

/* ========================================
   FOOTER
   ======================================== */
.credits-footer {
  text-align: center;
  color: #64748b;
  font-size: 0.9rem;
  padding: 2rem 0;
  border-top: 1px solid rgba(148, 163, 184, 0.1);
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
}

/* ========================================
   RESPONSIVE
   ======================================== */
@media (max-width: 768px) {
  .hero-title {
    font-size: 2.5rem;
  }

  .team-grid {
    grid-template-columns: 1fr;
  }

  .tech-grid {
    grid-template-columns: repeat(auto-fill, minmax(140px, 1fr));
  }

  .badges-grid {
    grid-template-columns: 1fr;
    max-width: 320px;
    margin: 0 auto;
  }

  .section-title {
    font-size: 1.5rem;
  }
}
</style>
