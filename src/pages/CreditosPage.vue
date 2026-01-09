<template>
  <q-page class="q-pa-lg column items-center bg-slate-dark">
    <!-- 1. HERO: PROYECTO UP-ROS -->
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

    <!-- 2. EQUIPO DE DESARROLLO (Cards Estilo Perfil) -->
    <div class="section-group self-stretch">
      <SectionTitle>Equipo de Desarrollo</SectionTitle>

      <div class="team-grid">
        <!-- TARJETA 1: ALEXANDER (AUTOR) -->
        <div class="profile-card">
          <div class="profile-img-container">
            <img src="~assets/images/alex.png" alt="Alexander Calderon" class="profile-img" />
          </div>

          <div class="profile-content">
            <h2 class="text-h5 text-white q-my-sm">Alexander Calderon Leal</h2>
            <div class="text-subtitle2 text-primary text-weight-bold">Ingeniero Mecatrónico</div>
            <div class="text-caption text-grey-4 q-mb-md">Universidad de Pamplona</div>

            <p class="text-grey-4 text-body2 text-justify">
              Especializado en la integración de mecánica, electrónica y software. Enfocado en
              robótica móvil (ROS 2), simulación y arquitecturas escalables. Creador de la
              arquitectura UP-Ros.
            </p>

            <!-- Botones Sociales -->
            <div class="row justify-center q-gutter-sm q-mt-md">
              <q-btn
                flat
                round
                dense
                color="white"
                icon="code"
                href="https://github.com/"
                target="_blank"
              >
                <q-tooltip>GitHub</q-tooltip>
              </q-btn>
              <q-btn
                flat
                round
                dense
                color="blue-5"
                icon="work"
                href="https://linkedin.com/"
                target="_blank"
              >
                <q-tooltip>LinkedIn</q-tooltip>
              </q-btn>
              <q-btn
                flat
                round
                dense
                color="green-4"
                icon="mail"
                href="mailto:alexander@example.com"
              >
                <q-tooltip>Contacto</q-tooltip>
              </q-btn>
            </div>
          </div>
        </div>

        <!-- TARJETA 2: DIRECTOR / COLABORADOR -->
        <!-- (Edita los datos aquí abajo) -->
        <div class="profile-card">
          <div class="profile-img-container">
            <!-- Usa una foto real si tienes, si no, deja este icono -->
            <q-icon name="person" size="60px" color="grey-4" class="placeholder-icon" />
          </div>

          <div class="profile-content">
            <h2 class="text-h5 text-white q-my-sm">[Nombre del Director]</h2>
            <div class="text-subtitle2 text-secondary text-weight-bold">Director de Proyecto</div>
            <div class="text-caption text-grey-4 q-mb-md">Docente - Facultad de Ingenierías</div>

            <p class="text-grey-4 text-body2 text-justify">
              [Espacio para describir brevemente el rol del director, su especialidad académica (ej:
              Control Automático, Inteligencia Artificial) y su guía fundamental en el desarrollo
              metodológico de este proyecto.]
            </p>

            <div class="row justify-center q-gutter-sm q-mt-md">
              <q-btn flat round dense color="white" icon="school">
                <q-tooltip>CvLAC / Perfil Académico</q-tooltip>
              </q-btn>
              <q-btn flat round dense color="white" icon="mail">
                <q-tooltip>Correo Institucional</q-tooltip>
              </q-btn>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. STACK TECNOLÓGICO (Resumido) -->
    <div class="section-group self-stretch">
      <SectionTitle>Tecnologías Clave</SectionTitle>
      <div class="tech-category q-mt-lg">
        <div class="tech-grid">
          <div v-for="tech in stackRobotics" :key="tech.name" class="tech-item">
            <q-icon :name="tech.icon" size="md" :color="tech.color" />
            <div class="tech-name">{{ tech.name }}</div>
            <div class="tech-role">{{ tech.role }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. RESPALDO INSTITUCIONAL (Mezcla Premium: Elogio + Logos Detallados) -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>Sello de Calidad Académica</SectionTitle>

      <div class="institutional-container">
        <!-- A. Encabezado de Reconocimiento -->
        <div class="text-center q-mb-xl intro-institutional">
          <h3 class="text-h4 text-white q-mb-sm">Universidad de Pamplona</h3>
          <div class="text-overline text-grey-4 q-mb-md tracking-wide">FUNDADA EN 1960</div>

          <p class="text-body1 text-grey-4 citation-text">
            "Formando nuevas generaciones con sello de excelencia comprometidos<br />
            con la transformación social de las regiones y un país en paz"
          </p>

          <TextBlock class="q-mt-md text-justify-center box-text-limit">
            Este proyecto es el resultado de la formación integral recibida en una institución con
            <strong>Acreditación Institucional de Alta Calidad</strong>. El programa de Ingeniería
            Mecatrónica se destaca por su compromiso con la excelencia, la investigación aplicada y
            la proyección internacional.
          </TextBlock>
        </div>

        <!-- B. Grid de Badges (Logos con Descripción) -->
        <div class="badges-grid">
          <!-- 1. ALMA MATER -->
          <div class="badge-card">
            <div class="logo-box">
              <img src="~assets/images/escudounipamplona.png" alt="Escudo Unipamplona" />
            </div>
            <div class="badge-info">
              <div class="badge-title">Alma Mater</div>
              <div class="badge-desc">Tradición y Excelencia Académica</div>
            </div>
          </div>

          <!-- 2. PROGRAMA -->
          <div class="badge-card">
            <div class="logo-box">
              <img src="~assets/images/Mecatronica.png" alt="Ingeniería Mecatrónica" />
            </div>
            <div class="badge-info">
              <div class="badge-title">Ingeniería Mecatrónica</div>
              <div class="badge-desc">Innovación y Desarrollo Tecnológico</div>
            </div>
          </div>

          <!-- 3. CALIDAD -->
          <div class="badge-card">
            <div class="logo-box">
              <img src="~assets/images/logo_acreditacion_png.png" alt="CNA Acreditación" />
            </div>
            <div class="badge-info">
              <div class="badge-title">Alta Calidad</div>
              <div class="badge-desc">Reconocimiento del Ministerio de Educación</div>
            </div>
          </div>

          <!-- 4. INTERNACIONAL -->
          <div class="badge-card">
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
    </div>

    <!-- FOOTER -->
    <div class="text-center text-grey-6 q-pb-xl text-caption">
      &copy; {{ new Date().getFullYear() }} Alexander Calderon Leal. Desarrollado en Colombia.
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';

// STACK ROBÓTICA (Solo lo más relevante)
const stackRobotics = [
  { name: 'ROS 2 Jazzy', role: 'Middleware', icon: 'smart_toy', color: 'purple-5' },
  { name: 'Gazebo', role: 'Simulación', icon: 'public', color: 'orange-5' },
  { name: 'Nav2', role: 'Navegación', icon: 'explore', color: 'green-5' },
  { name: 'Vue & Quasar', role: 'Interfaz HMI', icon: 'layers', color: 'blue-5' },
];
</script>

<style scoped>
/* --- BASE --- */
.bg-slate-dark {
  background-color: #0f172a;
  min-height: 100vh;
  color: #f8fafc;
}
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}
.text-justify-center {
  text-align: justify;
  text-align-last: center;
} /* Truco para centrar última línea */

/* --- HERO --- */
.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at bottom, rgba(139, 92, 246, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
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

/* --- TEAM GRID --- */
.team-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); /* Tarjetas más anchas */
  gap: 24px;
}

.profile-card {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  padding: 30px;
  text-align: center;
  transition:
    transform 0.3s ease,
    border-color 0.3s ease;
  display: flex;
  flex-direction: column;
  align-items: center;
}

.profile-card:hover {
  transform: translateY(-5px);
  background: rgba(30, 41, 59, 0.6);
  border-color: rgba(56, 189, 248, 0.4);
}

.profile-img-container {
  width: 120px;
  height: 120px;
  border-radius: 50%;
  padding: 3px;
  background: linear-gradient(135deg, #38bdf8, #818cf8); /* Anillo gradiente */
  margin-bottom: 1.2rem;
  box-shadow: 0 10px 20px rgba(0, 0, 0, 0.3);
}

.profile-img {
  width: 100%;
  height: 100%;
  object-fit: cover;
  border-radius: 50%;
  border: 3px solid #0f172a; /* Borde interno para separar del anillo */
}

.placeholder-icon {
  width: 100%;
  height: 100%;
  background: #1e293b;
  border-radius: 50%;
  border: 3px solid #0f172a;
  display: flex;
  align-items: center;
  justify-content: center;
}

/* --- TECH GRID --- */
.tech-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
  gap: 16px;
}
.tech-item {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  padding: 16px;
  border-radius: 12px;
  text-align: center;
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

/* --- INSTITUCIONAL PREMIUM --- */
.institutional-container {
  background: linear-gradient(180deg, rgba(30, 41, 59, 0.3) 0%, rgba(15, 23, 42, 0.6) 100%);
  border: 1px solid rgba(255, 255, 255, 0.05);
  border-radius: 24px;
  padding: 3rem 2rem;
  position: relative;
  overflow: hidden;
}

/* Línea superior decorativa (Rojo Unipamplona aprox) */
.institutional-container::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  height: 4px;
  background: linear-gradient(90deg, #aa1c3a, #d90429, #aa1c3a);
  opacity: 0.8;
}

.citation-text {
  font-style: italic;
  font-family: 'Times New Roman', serif;
  color: #e2e8f0;
  font-size: 1.1rem;
}
.box-text-limit {
  max-width: 800px;
  margin: 0 auto;
}

/* Grid de Badges */
.badges-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  gap: 24px;
  margin-top: 2.5rem;
}

.badge-card {
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  background: rgba(255, 255, 255, 0.02);
  border-radius: 16px;
  padding: 20px;
  border: 1px solid transparent;
  transition: all 0.3s ease;
}

.badge-card:hover {
  transform: translateY(-5px);
  background: rgba(255, 255, 255, 0.05);
  border-color: rgba(255, 255, 255, 0.1);
}

.logo-box {
  background: white; /* Fondo blanco para respetar colores originales de los logos */
  width: 100%;
  height: 90px;
  border-radius: 12px;
  display: flex;
  justify-content: center;
  align-items: center;
  padding: 12px;
  margin-bottom: 16px;
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1);
}

.logo-box img {
  max-width: 100%;
  max-height: 100%;
  object-fit: contain;
}

.badge-title {
  color: #f8fafc;
  font-weight: 700;
  font-size: 1.05rem;
  margin-bottom: 4px;
}
.badge-desc {
  color: #94a3b8;
  font-size: 0.85rem;
  line-height: 1.4;
}

/* RESPONSIVE */
@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .badges-grid {
    grid-template-columns: 1fr;
    max-width: 320px;
    margin-inline: auto;
  }
  .team-grid {
    grid-template-columns: 1fr;
  }
}
</style>
