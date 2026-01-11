<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. AUTO-HEADER: Estilo Premium (Reference: 04serviciosPage) -->
    <section class="intro-hero self-stretch" :style="heroStyle">
      <div v-if="currentNode" class="hero-content">
        <!-- Breadcrumb / Overline -->
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          {{ parentModule?.title || 'LECCIÓN' }}
        </div>

        <h1 class="hero-title">
          {{ currentNode.title }}
        </h1>

        <div v-if="currentNode.description" class="hero-desc">
          <p>{{ currentNode.description }}</p>
        </div>
      </div>

      <!-- Fallback Skeleton -->
      <div v-else class="header-skeleton column items-center">
        <q-skeleton type="text" width="30%" class="q-mb-sm" />
        <q-skeleton type="rect" height="60px" width="80%" />
      </div>
    </section>

    <!-- 2. CONTENIDO PRINCIPAL (Slot) -->
    <div class="lesson-content self-stretch">
      <slot />
    </div>

    <!-- 3. FOOTER DE NAVEGACIÓN -->
    <div class="lesson-footer self-stretch row justify-between q-mt-xl">
      <!-- Botón ANTERIOR -->
      <q-btn v-if="prevNode" flat no-caps class="nav-btn prev" :to="getRoutePath(prevNode)">
        <div class="column items-start text-left">
          <div class="text-caption text-grey-6">Anterior</div>
          <div class="text-weight-bold text-white">{{ prevNode.node.title }}</div>
        </div>
        <q-icon name="arrow_back" class="on-left q-mr-md" />
      </q-btn>
      <div v-else></div>

      <!-- Botón SIGUIENTE -->
      <q-btn v-if="nextNode" flat no-caps class="nav-btn next" :to="getRoutePath(nextNode)">
        <div class="column items-end text-right">
          <div class="text-caption text-grey-6">Siguiente</div>
          <div class="text-weight-bold text-white">{{ nextNode.node.title }}</div>
        </div>
        <q-icon name="arrow_forward" class="on-right q-ml-md" />
      </q-btn>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import { computed } from 'vue';
import { useRoute } from 'vue-router';
import { courseStructure, type CourseNode } from 'src/data/courseStructure';

const route = useRoute();

// --- 1. ENCONTRAR NODO ACTUAL ---
const flattenedNodes = computed(() => {
  const nodes: { node: CourseNode; parent: CourseNode | null; fullPath: string }[] = [];

  courseStructure.forEach((module) => {
    if (module.children) {
      module.children.forEach((child) => {
        const modulePath = module.path || '';
        const childPath = child.path || '';

        nodes.push({
          node: child,
          parent: module,
          fullPath: `/${modulePath}/${childPath}`,
        });
      });
    }
  });
  return nodes;
});

const currentIndex = computed(() => {
  return flattenedNodes.value.findIndex((item) => item.fullPath === route.path);
});

const currentItem = computed(() =>
  currentIndex.value !== -1 ? flattenedNodes.value[currentIndex.value] : null,
);

const currentNode = computed(() => currentItem.value?.node);
const parentModule = computed(() => currentItem.value?.parent);

// --- 2. HERO STYLE DINÁMICO ---
const heroStyle = computed(() => {
  const path = route.path;
  let color = '56, 189, 248'; // Default Blue (Tailwind sky-400 / Quasar blue-5 approx)

  if (path.includes('modulo-0'))
    color = '76, 175, 80'; // Green-5
  else if (path.includes('modulo-1'))
    color = '156, 39, 176'; // Purple-5
  else if (path.includes('modulo-2'))
    color = '33, 150, 243'; // Blue-5
  else if (path.includes('modulo-3'))
    color = '255, 193, 7'; // Amber-5
  else if (path.includes('modulo-4'))
    color = '244, 67, 54'; // Red-5
  else if (path.includes('modulo-5'))
    color = '0, 188, 212'; // Cyan-5
  else if (path.includes('modulo-6'))
    color = '255, 152, 0'; // Orange-5
  else if (path.includes('modulo-7'))
    color = '63, 81, 181'; // Indigo-5
  else if (path.includes('modulo-8')) color = '0, 150, 136'; // Teal-5

  return {
    background: `radial-gradient(circle at center, rgba(${color}, 0.15), transparent 60%), rgba(15, 23, 42, 0.8)`,
  };
});

// --- 3. NAVEGACIÓN ---
const prevNode = computed(() => {
  if (currentIndex.value > 0) return flattenedNodes.value[currentIndex.value - 1];
  return null;
});

const nextNode = computed(() => {
  if (currentIndex.value !== -1 && currentIndex.value < flattenedNodes.value.length - 1) {
    return flattenedNodes.value[currentIndex.value + 1];
  }
  return null;
});

const getRoutePath = (item: { fullPath: string } | null) => {
  return item ? item.fullPath : '/';
};
</script>

<style scoped lang="scss">
/* Estilos alineados con 04serviciosPage.vue */

/* HERO SECTION */
.intro-hero {
  width: 100%;
  max-width: 1100px; /* Ancho máximo igual a la referencia */
  margin: 0 auto 3.5rem auto; /* Centrado horizontal */
  padding: 3rem 2rem;

  /* background is now dynamic via :style */

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

.hero-desc {
  font-size: 1.1rem;
  max-width: 800px;
  margin: 0 auto; /* Centrado del texto */
  line-height: 1.6;
  color: #cbd5e1;
}

/* CONTENT CONTAINER */
.lesson-content {
  width: 100%;
  max-width: 1100px; /* Mismo ancho que el Hero */
  margin: 0 auto; /* CRUCIAL: Centrado del bloque de contenido */
  display: flex;
  flex-direction: column;
  align-items: center; /* Centra los hijos si son flex items */
}

/* Deep selector para forzar que los hijos directos (como .section-group) también respeten el ancho */
.lesson-content :deep(.section-group) {
  width: 100%;
  max-width: 100%; /* Ya está restringido por el padre */
}

/* FOOTER NAV */
.lesson-footer {
  width: 100%;
  max-width: 1100px;
  margin: 3rem auto 0 auto; /* Centrado horizontal */
  border-top: 1px solid rgba(255, 255, 255, 0.05);
  padding-top: 2rem;
}

.nav-btn {
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 12px;
  padding: 12px 24px;
  transition: all 0.3s ease;
  background: rgba(30, 41, 59, 0.4);

  &:hover {
    background: rgba(56, 189, 248, 0.15);
    border-color: rgba(56, 189, 248, 0.5);
    transform: translateY(-2px);
  }
}

/* RESPONSIVE */
@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .intro-hero {
    padding: 2rem 1.5rem;
  }
}
</style>
