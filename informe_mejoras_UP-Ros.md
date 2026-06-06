# 📊 Informe de Análisis y Mejoras — UP-Ros

> **Stack**: Quasar v2 (Vue 3) + Vite + TypeScript + Electron  
> **Propósito**: Plataforma educativa de ROS 2 Jazzy (Desktop App + PWA)  
> **Análisis**: Cobertura total del código fuente — componentes, rutas, datos, estilos y proceso Electron

---

## Resumen Ejecutivo

El proyecto está bien estructurado para una primera versión y tiene decisiones de diseño sólidas (arquitectura data-driven, lazy loading de rutas, glassmorphism consistente). Hay **9 áreas de mejora** identificadas, clasificadas por impacto.

| Prioridad | Categoría | Descripción |
|---|---|---|
| 🔴 CRÍTICO | **Seguridad (Electron)** | `webSecurity: false` deshabilitado globalmente |
| 🔴 CRÍTICO | **XSS Vulnerability** | `v-html` sin sanitización en `GlosarioPage` |
| 🟠 ALTO | **Mantenibilidad** | Duplicación masiva de lógica de colores en 4 archivos |
| 🟠 ALTO | **Performance** | `flattenedNodes` recalcula en cada render de lección |
| 🟠 ALTO | **Routing** | Bug potencial en `replace('-', '')` — solo reemplaza el primero |
| 🟡 MEDIO | **UX** | Sin feedback de progreso del curso (sin Pinia/store) |
| 🟡 MEDIO | **Accesibilidad** | Falta de `aria-*` en elementos interactivos clave |
| 🟢 BAJO | **TypeScript** | Tipos parciales en `courseStructure` — sin gestión de estado global |
| 🟢 BAJO | **CSS** | Valores mágicos repetidos, sin sistema de design tokens |

---

## 🔴 1. Vulnerabilidad de Seguridad Crítica — `webSecurity: false`

**Archivo**: `src-electron/electron-main.ts` → L23

```typescript
// ❌ ACTUAL — Desactiva toda la política de seguridad del navegador
webSecurity: false,
```

**Problema**: Esta configuración desactiva la misma política de seguridad que protege al usuario de ataques XSS, inyección de contenido y carga de recursos maliciosos. El comentario dice "_es la ÚNICA solución para iframes de YouTube_", pero no lo es.

**Solución correcta**:
```typescript
// ✅ CORRECTO — Usar Content Security Policy selectiva
// 1. Mantener webSecurity: true (eliminar la línea, es el default)
webPreferences: {
  contextIsolation: true,
  nodeIntegration: false,
  // No webSecurity: false
},

// 2. Configurar CSP en el BrowserWindow
mainWindow.webContents.session.webRequest.onHeadersReceived((details, callback) => {
  callback({
    responseHeaders: {
      ...details.responseHeaders,
      'Content-Security-Policy': [
        "default-src 'self'; frame-src https://www.youtube.com https://www.youtube-nocookie.com; img-src 'self' data: https:; style-src 'self' 'unsafe-inline';"
      ]
    }
  });
});
```

> [!CAUTION]
> Esta es la vulnerabilidad más grave del proyecto. Un atacante podría explotar contenido externo cargado en la app para ejecutar código arbitrario con privilegios del sistema operativo.

---

## 🔴 2. Vulnerabilidad XSS — `v-html` sin sanitizar

**Archivo**: `src/pages/GlosarioPage.vue` → L82, L102, L163-164

```typescript
// ❌ ACTUAL — Inyecta HTML directamente desde datos externos
const highlightText = (text: string) => {
  const regex = new RegExp(`(${search.value})`, 'gi'); // ⚠️ regexes sin escape
  return text.replace(regex, '<span class="highlight">$1</span>');
};
```

```html
<!-- ❌ ACTUAL -->
<div class="term-name" v-html="highlightText(term.name)"></div>
```

**Dos problemas**:
1. El `search.value` del usuario no está escapado al construir el RegExp → ReDoS (Regular Expression Denial of Service)
2. El HTML generado se inyecta sin sanitización

**Solución**:
```typescript
// ✅ CORRECTO
import DOMPurify from 'dompurify'; // npm install dompurify @types/dompurify

const escapeRegex = (str: string) => str.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');

const highlightText = (text: string) => {
  if (!search.value) return text;
  const safe = escapeRegex(search.value);
  const regex = new RegExp(`(${safe})`, 'gi');
  const highlighted = text.replace(regex, '<mark class="highlight">$1</mark>');
  return DOMPurify.sanitize(highlighted); // Sanitizar antes de inyectar
};
```

---

## 🟠 3. Bug en el Generador de Rutas — `replace('-', '')`

**Archivo**: `src/router/routes.ts` → L20

```typescript
// ❌ ACTUAL — String.replace() solo reemplaza la PRIMERA coincidencia
const folderName = node.path?.replace('-', '') || '';
// 'modulo-0' → 'modulo0'  ✅ Funciona por casualidad
// 'modulo-10' → 'modulo10' ✅ OK
// 'super-modulo-0' → 'supermodulo-0' ❌ BUG: solo el primer guion
```

**Solución**:
```typescript
// ✅ CORRECTO — Usar RegExp global para reemplazar TODOS los guiones
const folderName = node.path?.replace(/-/g, '') || '';
```

> [!WARNING]
> Aunque no explota con los módulos actuales, escalar el curso con rutas como `modulo-de-ros` causaría paths incorrectos silenciosamente.

---

## 🟠 4. Duplicación masiva — Sistema de Colores

**Archivos afectados**: `SideMenu.vue`, `LessonContainer.vue`, `HomePage.vue` (3 veces)

```typescript
// ❌ ACTUAL — Lógica de color DUPLICADA en 3+ archivos
// SideMenu.vue:
function getModuleColorClass(path: string): string {
  if (path.includes('modulo-0')) return 'text-green-4';
  if (path.includes('modulo-1')) return 'text-purple-4';
  // ...8 condiciones más
}

// LessonContainer.vue (misma lógica con RGB):
if (path.includes('modulo-0')) color = '76, 175, 80';
if (path.includes('modulo-1')) color = '156, 39, 176';
// ...8 condiciones más

// HomePage.vue (misma lógica 3 veces):
const getModuleBgColor = (path: string) => { ... }   // 9 condiciones
const getModuleTextColor = (path: string) => { ... } // 9 condiciones
const getChipColor = (path: string) => { ... }       // 8 condiciones
```

**Solución — Crear un `composable` centralizado**:

```typescript
// ✅ NUEVO: src/composables/useModuleColors.ts

export const MODULE_COLORS: Record<string, {
  text: string;
  bg: string;
  chip: string;
  rgb: string;
  border: string;
}> = {
  'modulo-0': { text: 'text-green-4',  bg: 'bg-green-9',  chip: 'green-2',  rgb: '76, 175, 80',   border: 'bg-green-5' },
  'modulo-1': { text: 'text-purple-4', bg: 'bg-purple-9', chip: 'purple-2', rgb: '156, 39, 176',  border: 'bg-purple-5' },
  'modulo-2': { text: 'text-blue-4',   bg: 'bg-blue-9',   chip: 'cyan-2',   rgb: '33, 150, 243',  border: 'bg-blue-5' },
  'modulo-3': { text: 'text-amber-4',  bg: 'bg-amber-9',  chip: 'amber-2',  rgb: '255, 193, 7',   border: 'bg-amber-5' },
  'modulo-4': { text: 'text-red-4',    bg: 'bg-red-9',    chip: 'red-2',    rgb: '244, 67, 54',   border: 'bg-red-5' },
  'modulo-5': { text: 'text-cyan-4',   bg: 'bg-indigo-9', chip: 'white',    rgb: '0, 188, 212',   border: 'bg-cyan-5' },
  'modulo-6': { text: 'text-orange-4', bg: 'bg-orange-9', chip: 'orange-2', rgb: '255, 152, 0',   border: 'bg-orange-5' },
  'modulo-7': { text: 'text-indigo-4', bg: 'bg-pink-9',   chip: 'white',    rgb: '63, 81, 181',   border: 'bg-indigo-5' },
  'modulo-8': { text: 'text-teal-4',   bg: 'bg-green-9',  chip: 'green-2',  rgb: '0, 150, 136',   border: 'bg-teal-5' },
};

export function useModuleColors() {
  const getColors = (path: string) => {
    const key = Object.keys(MODULE_COLORS).find(k => path.includes(k));
    return key ? MODULE_COLORS[key] : { text: 'text-grey-4', bg: 'bg-grey-8', chip: 'white', rgb: '100, 116, 139', border: 'bg-grey-6' };
  };

  return { getColors };
}
```

**Beneficio**: Modificar el color de un módulo requiere cambiar **1 línea**, no **5 archivos**.

---

## 🟠 5. Performance — `flattenedNodes` sin memoización eficiente

**Archivo**: `src/components/content/LessonContainer.vue` → L64-82

```typescript
// ⚠️ ACTUAL — Se recalcula con cada cambio de ruta (correcto con computed),
// pero el problema es que es O(n*m) en el template de cada lección
const flattenedNodes = computed(() => {
  const nodes = [];
  courseStructure.forEach((module) => {       // n módulos
    if (module.children) {
      module.children.forEach((child) => {    // m lecciones por módulo
        nodes.push({ node: child, parent: module, fullPath: `/${module.path}/${child.path}` });
      });
    }
  });
  return nodes;
});
```

**El verdadero problema**: `flattenedNodes` es idéntico en CADA instancia de `LessonContainer`. Se recalcula una vez por página visitada.

**Solución — Mover a nivel de módulo (hoisting)**:
```typescript
// ✅ CORRECTO: src/composables/useCourseNavigation.ts
import { courseStructure } from 'src/data/courseStructure';

// Se calcula UNA SOLA VEZ al cargar el módulo
const flattenedNodes = courseStructure.flatMap(module =>
  (module.children ?? []).map(child => ({
    node: child,
    parent: module,
    fullPath: `/${module.path}/${child.path}`,
  }))
);

export function useCourseNavigation() {
  // Solo el índice es reactivo y depende de la ruta
  return { flattenedNodes };
}
```

---

## 🟡 6. Sin Estado de Progreso del Usuario (Sin Pinia)

**Problema actual**: El usuario no puede ver qué lecciones ha completado, no hay estado persistente. El `CourseNode` tiene el campo `prerequisite` pero **no hay lógica que lo consuma**.

**Solución — Agregar Pinia** (ya está disponible con Quasar):

```typescript
// ✅ NUEVO: src/stores/progressStore.ts
import { defineStore } from 'pinia';
import { ref, computed } from 'vue';

export const useProgressStore = defineStore('progress', () => {
  // Persistido en localStorage
  const completedPaths = ref<Set<string>>(
    new Set(JSON.parse(localStorage.getItem('completed') ?? '[]'))
  );

  function markComplete(fullPath: string) {
    completedPaths.value.add(fullPath);
    localStorage.setItem('completed', JSON.stringify([...completedPaths.value]));
  }

  const totalLessons = computed(() => /* contar de courseStructure */ 0);
  const completedCount = computed(() => completedPaths.value.size);
  const progress = computed(() => Math.round((completedCount.value / totalLessons.value) * 100));

  return { completedPaths, markComplete, progress };
}, { persist: true }); // Con pinia-plugin-persistedstate
```

**Impacto de UX**: Agregar un simple `ProgressBar` al `SideMenu` sería un diferenciador clave para la plataforma educativa.

---

## 🟡 7. Accesibilidad (a11y) — Deficiencias

### 7.1 — Módulos sin `aria-expanded`
```html
<!-- ❌ ACTUAL: El lector de pantalla no sabe si el acordeón está abierto -->
<q-expansion-item group="modules" class="module-expansion">

<!-- ✅ CORRECTO -->
<q-expansion-item
  :aria-expanded="isModuleOpen(module.path)"
  :aria-label="`Módulo: ${module.title}`"
>
```

### 7.2 — Íconos decorativos sin `aria-hidden`
```html
<!-- ❌ ACTUAL: El lector leerá "terminal icon" innecesariamente -->
<q-icon name="terminal" />

<!-- ✅ CORRECTO -->
<q-icon name="terminal" aria-hidden="true" />
```

### 7.3 — Marcador de posición activa en menú
```html
<!-- ✅ Agregar aria-current para la lección activa -->
<q-item :aria-current="isActive ? 'page' : undefined">
```

### 7.4 — `lang` en el `<html>`
```html
<!-- ❌ ACTUAL: index.html no tiene lang declarado -->
<!-- ✅ CORRECTO -->
<html lang="es">
```

---

## 🟢 8. TypeScript — Mejoras de Tipado

### 8.1 — `getModuleDescription()` desincronizada con `courseStructure`
```typescript
// ❌ ACTUAL: descripciones hardcodeadas in HomePage.vue, NO en courseStructure
// Pero courseStructure TIENE description en algunos nodos. Inconsistencia.
const getModuleDescription = (path: string) => {
  const descriptions: Record<string, string> = {
    'modulo-0': 'Fundamentos de Linux...', // ← Debería venir del dato
  };
};
```

**Solución**: Agregar `description` al nivel de módulo en `CourseNode` y elimininar el diccionario hardcodeado:
```typescript
// En courseStructure.ts — agregar 'description' al módulo (ya existe en children)
{
  title: 'Módulo 0: Fundamentos Linux',
  description: 'Domina la línea de comandos Linux: navegación, permisos y gestión de paquetes.',
  // ...
}

// En HomePage.vue — simplificar a:
const getModuleDescription = (module: CourseNode) => module.description ?? 'Robótica avanzada.';
```

### 8.2 — `breadcrumbs` en `MainLayout.vue` usa `any` implícito
```typescript
const crumbs = []; // ❌ Tipo inferido: any[]

// ✅ CORRECTO
const crumbs: { label: string; to: string; icon?: string }[] = [];
```

### 8.3 — `import` fuera del `<script setup>`
```typescript
// ❌ ACTUAL en SideMenu.vue (L235) — import dentro del bloque de código
import { watch } from 'vue';
import { useRoute } from 'vue-router';

// ✅ CORRECTO — Los imports van al inicio del <script setup>
```

---

## 🟢 9. CSS y Design Tokens

### 9.1 — Valores Mágicos sin Variables CSS

Los colores del glassmorphism están hardcodeados en **múltiples archivos**:

```scss
// ❌ Repetido en MainLayout.vue, SideMenu.vue, app.scss...
background: rgba(15, 23, 42, 0.75);
background: rgba(15, 25, 40, 0.6);
background: rgba(15, 23, 42, 0.98);
// ¿Cuál es el correcto? No hay consistencia
```

**Solución — Design tokens en `:root`**:
```css
/* ✅ app.scss — una fuente de verdad */
:root {
  /* Paleta Base */
  --color-bg-page: #0f172a;
  --color-bg-surface: rgba(30, 41, 59, 0.4);
  --color-bg-surface-hover: rgba(30, 41, 59, 0.8);
  --color-border: rgba(255, 255, 255, 0.08);
  --color-border-hover: rgba(56, 189, 248, 0.3);

  /* Glassmorphism */
  --glass-bg: rgba(15, 23, 42, 0.75);
  --glass-blur: blur(18px);

  /* Accent */
  --color-accent: #38bdf8;
  --color-accent-rgb: 56, 189, 248;

  /* Tipografía */
  --font-mono: 'Fira Code', 'Monaco', 'Consolas', monospace;
}
```

### 9.2 — Animación `slideIn` en menú re-lanza en cada render
```css
/* ❌ ACTUAL — Se anima CADA ítem con cada render del componente */
.menu-item, .module-expansion {
  animation: slideIn 0.3s ease-out; /* Sin fill-mode ni delay progresivo */
}

/* ✅ CORRECTO — Sólo al montar, con delays escalonados */
.menu-item:nth-child(n) {
  animation: slideIn 0.3s ease-out both;
  animation-delay: calc(var(--item-index, 0) * 50ms);
}
```

### 9.3 — `z-index: 0.3` no es válido
```css
/* ❌ ACTUAL en HomePage.vue — z-index acepta solo enteros */
.hero-robot {
  z-index: 0.3; /* Se ignora silenciosamente */
}

/* ✅ CORRECTO */
.hero-robot {
  z-index: 1;
}
```

---

## 🔵 Mejoras de Funcionalidad Propuestas

Estas no son bugs sino features de alto valor con esfuerzo razonable:

| Feature | Impacto | Esfuerzo |
|---|---|---|
| **Progreso persistente** (Pinia + localStorage) | ⭐⭐⭐⭐⭐ | Medio |
| **Syntax highlighting en CodeBlock** (Shiki/Prism) | ⭐⭐⭐⭐ | Bajo |
| **Búsqueda global** con `Ctrl+K` (Command Palette) | ⭐⭐⭐⭐ | Medio |
| **Modo offline completo** (PWA ya configurado) | ⭐⭐⭐ | Bajo |
| **Prerequisitos visuales** (bloquear módulos según orden) | ⭐⭐⭐ | Alto |
| **Favoritos de lecciones** | ⭐⭐ | Bajo |

---

## 📁 Estructura de Archivos Propuesta

```
src/
├── composables/           # ← NUEVO
│   ├── useModuleColors.ts  # Colores centralizados
│   └── useCourseNavigation.ts  # flattenedNodes + prev/next
├── stores/                # ← NUEVO
│   └── progressStore.ts   # Progreso del usuario (Pinia)
├── data/
│   ├── courseStructure.ts  # Agregar description a módulos
│   └── glossaryTerms.ts
├── components/
│   └── content/
│       └── CodeBlock.vue  # Agregar syntax highlighting
...
```

---

## Summary de Impacto

```
🔴 Seguridad     → 2 issues críticos (webSecurity + XSS)
🟠 Bugs         → 1 bug funcional (route generator)
🟠 Performance  → 1 optimización clave (flattenedNodes hoisting)
🟠 Mantenibilidad → Eliminar ~150 líneas duplicadas con 1 composable
🟡 UX            → Progreso del curso (Pinia)
🟡 a11y          → 4 fixes de accesibilidad
🟢 TypeScript    → 3 mejoras de tipado
🟢 CSS           → Design tokens + z-index fix
```

**Tiempo estimado de implementación**: Los issues críticos (seguridad) en ~2 horas. El composable de colores en ~1 hora. El store de progreso en ~3 horas. Total para todas las mejoras: ~1-2 días de desarrollo.
