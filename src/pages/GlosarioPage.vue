<template>
  <q-page class="q-pa-none bg-slate-dark">
    <!-- 1. HERO + CONTROLES -->
    <div class="header-wrapper">
      <section class="intro-hero">
        <div class="hero-content">
          <div class="text-overline text-accent text-weight-bold tracking-widest">
            DICCIONARIO TÉCNICO
          </div>
          <h1 class="hero-title">Glosario <span class="text-gradient">ROS 2</span></h1>
          <p class="hero-desc">La enciclopedia definitiva para tu carrera robótica.</p>
        </div>
      </section>

      <div class="toolbar-sticky q-px-md q-py-sm">
        <div class="toolbar-content">
          <q-input
            v-model="search"
            dark
            dense
            standout
            placeholder="Buscar concepto..."
            class="search-input"
          >
            <template v-slot:prepend><q-icon name="search" /></template>
            <template v-slot:append>
              <q-icon v-if="search" name="close" class="cursor-pointer" @click="search = ''" />
            </template>
          </q-input>

          <div class="filters-scroll">
            <q-chip
              v-for="cat in categories"
              :key="cat"
              clickable
              :color="selectedCategory === cat ? getCategoryColor(cat) : 'blue-grey-9'"
              :text-color="selectedCategory === cat ? 'white' : 'grey-4'"
              :icon="selectedCategory === cat ? 'check' : undefined"
              @click="toggleCategory(cat)"
              class="filter-chip"
            >
              {{ cat === 'All' ? 'Todos' : cat }}
            </q-chip>
          </div>
        </div>
      </div>
    </div>

    <!-- 2. RESULTADOS -->
    <div class="q-pa-lg content-width">
      <!-- CORRECCIÓN CLAVE AQUÍ ABAJO -->
      <!-- Usamos mode="out-in" para que una animación termine antes de empezar la otra -->
      <transition name="fade" mode="out-in">
        <!-- OPCIÓN A: No hay resultados (v-if) -->
        <div v-if="filteredTerms.length === 0" key="empty" class="empty-state">
          <div class="glitch-icon">¯\_(ツ)_/¯</div>
          <div class="text-h5 q-mt-md">Sin resultados</div>
          <p class="text-grey">
            No encontramos nada para "<strong>{{ search }}</strong
            >" en {{ selectedCategory }}
          </p>
          <q-btn outline color="primary" label="Limpiar Filtros" @click="resetFilters" />
        </div>

        <!-- OPCIÓN B: Hay resultados (v-else) -->
        <div v-else key="results">
          <!-- Contador -->
          <div class="text-grey-6 q-mb-md text-right text-caption">
            Mostrando {{ filteredTerms.length }} definiciones
          </div>

          <!-- Grid -->
          <div class="glossary-grid">
            <transition-group name="list">
              <div
                v-for="term in filteredTerms"
                :key="term.name"
                class="glossary-card"
                :class="term.category"
              >
                <div class="card-header">
                  <div class="term-name" v-html="highlightText(term.name)"></div>
                  <div class="row items-center q-gutter-sm">
                    <q-btn
                      flat
                      round
                      dense
                      size="sm"
                      icon="content_copy"
                      color="grey-6"
                      class="copy-btn"
                      @click="copyToClipboard(term)"
                    >
                      <q-tooltip>Copiar Definición</q-tooltip>
                    </q-btn>
                    <q-badge :color="getCategoryColor(term.category)" class="term-badge">
                      {{ term.category }}
                    </q-badge>
                  </div>
                </div>

                <div class="term-def" v-html="highlightText(term.definition)"></div>

                <div class="term-context" v-if="term.context">
                  <q-icon name="terminal" size="xs" class="q-mr-xs text-secondary" />
                  <span class="text-code">{{ term.context }}</span>
                </div>
              </div>
            </transition-group>
          </div>
        </div>
      </transition>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';
import { useQuasar } from 'quasar';
import { glossaryTerms, type Category, type Term } from 'src/data/glossaryTerms';

const $q = useQuasar();

// --- ESTADO ---
const search = ref('');
const selectedCategory = ref<Category | 'All'>('All');
const categories: (Category | 'All')[] = ['All', 'Core', 'Comm', 'Tools', 'Sim', 'Math'];

// --- ACCIONES ---
const toggleCategory = (cat: Category | 'All') => {
  if (selectedCategory.value === cat && cat !== 'All') {
    selectedCategory.value = 'All';
  } else {
    selectedCategory.value = cat;
  }
};

const resetFilters = () => {
  search.value = '';
  selectedCategory.value = 'All';
};

const copyToClipboard = (term: Term) => {
  const text = `${term.name}: ${term.definition}`;
  void navigator.clipboard
    .writeText(text)
    .then(() => {
      $q.notify({
        message: 'Definición copiada',
        color: 'positive',
        icon: 'check',
        timeout: 1000,
        position: 'top',
      });
    })
    .catch((err) => {
      console.error(err);
    });
};

const highlightText = (text: string) => {
  if (!search.value) return text;
  const regex = new RegExp(`(${search.value})`, 'gi');
  return text.replace(regex, '<span class="highlight">$1</span>');
};

// --- FILTRADO ---
const filteredTerms = computed(() => {
  let result = glossaryTerms;

  if (selectedCategory.value !== 'All') {
    result = result.filter((t) => t.category === selectedCategory.value);
  }

  if (search.value) {
    const query = search.value.toLowerCase();
    result = result.filter(
      (t) => t.name.toLowerCase().includes(query) || t.definition.toLowerCase().includes(query),
    );
  }

  return result.sort((a, b) => a.name.localeCompare(b.name));
});

// --- UI HELPERS ---
function getCategoryColor(cat: string): string {
  switch (cat) {
    case 'Core':
      return 'purple-6';
    case 'Comm':
      return 'orange-8';
    case 'Tools':
      return 'cyan-7';
    case 'Sim':
      return 'green-6';
    case 'Math':
      return 'red-5';
    default:
      return 'grey';
  }
}
</script>

<style scoped>
/* BASE */
.bg-slate-dark {
  background-color: #0f172a;
  min-height: 100vh;
  color: #f8fafc;
}
.content-width {
  max-width: 1200px;
  margin: 0 auto;
}
.text-gradient {
  background: linear-gradient(to right, #38bdf8, #818cf8);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}

/* HERO SECTION */
.header-wrapper {
  background: #0f172a;
  position: relative;
  z-index: 100;
}
.intro-hero {
  padding: 4rem 1rem 2rem 1rem;
  text-align: center;
  background: radial-gradient(circle at top, rgba(56, 189, 248, 0.15) 0%, transparent 70%);
}
.hero-title {
  font-size: 3.5rem;
  font-weight: 900;
  line-height: 1.1;
  margin: 0;
  letter-spacing: -1px;
}
.hero-desc {
  color: #94a3b8;
  font-size: 1.1rem;
  max-width: 600px;
  margin: 1rem auto 0;
}

/* STICKY TOOLBAR */
.toolbar-sticky {
  position: sticky;
  top: 0;
  z-index: 50;
  background: rgba(15, 23, 42, 0.85);
  backdrop-filter: blur(12px);
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
  display: flex;
  justify-content: center;
}
.toolbar-content {
  width: 100%;
  max-width: 900px;
  display: flex;
  gap: 16px;
  align-items: center;
  flex-wrap: wrap;
}
.search-input {
  min-width: 250px;
  flex-grow: 1;
}
.filters-scroll {
  display: flex;
  gap: 8px;
  overflow-x: auto;
  padding-bottom: 2px;
  scrollbar-width: none;
}

/* GRID CARDS */
.glossary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
  gap: 20px;
}

.glossary-card {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  padding: 24px;
  display: flex;
  flex-direction: column;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  position: relative;
  overflow: hidden;
}

.glossary-card:hover {
  transform: translateY(-4px);
  background: rgba(30, 41, 59, 0.8);
  border-color: rgba(56, 189, 248, 0.3);
  box-shadow: 0 10px 30px -10px rgba(0, 0, 0, 0.5);
}

/* Colores de Borde Hover */
.glossary-card.Core:hover {
  border-top-color: #a855f7;
}
.glossary-card.Comm:hover {
  border-top-color: #f97316;
}
.glossary-card.Tools:hover {
  border-top-color: #06b6d4;
}
.glossary-card.Sim:hover {
  border-top-color: #22c55e;
}
.glossary-card.Math:hover {
  border-top-color: #ef4444;
}

/* Contenido Card */
.term-name {
  font-size: 1.35rem;
  font-weight: 700;
  color: white;
  letter-spacing: -0.5px;
}
.term-def {
  color: #cbd5e1;
  font-size: 0.95rem;
  line-height: 1.6;
  margin: 12px 0;
  flex-grow: 1;
}
.term-context {
  background: #1e293b;
  border-radius: 6px;
  padding: 8px 12px;
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
  color: #fbbf24;
  border-left: 3px solid rgba(255, 255, 255, 0.1);
  align-self: flex-start;
}

/* HIGHLIGHT TEXT STYLE */
:deep(.highlight) {
  background: rgba(251, 191, 36, 0.2);
  color: #fbbf24;
  font-weight: bold;
  border-radius: 2px;
  padding: 0 2px;
}

/* EMPTY STATE */
.empty-state {
  text-align: center;
  padding: 4rem 2rem;
}
.glitch-icon {
  font-size: 4rem;
  color: #475569;
  font-weight: bold;
}

/* ANIMACIONES */
.list-move,
.list-enter-active,
.list-leave-active {
  transition: all 0.4s ease;
}
.list-enter-from,
.list-leave-to {
  opacity: 0;
  transform: scale(0.95);
}
.list-leave-active {
  position: absolute;
}

/* ANIMACIÓN FADE ENTRE ESTADOS */
.fade-enter-active,
.fade-leave-active {
  transition: opacity 0.3s ease;
}
.fade-enter-from,
.fade-leave-to {
  opacity: 0;
}

/* MEDIA QUERIES */
@media (max-width: 600px) {
  .hero-title {
    font-size: 2.5rem;
  }
  .toolbar-content {
    flex-direction: column;
    align-items: stretch;
  }
  .filters-scroll {
    padding-bottom: 8px;
  }
}
</style>
