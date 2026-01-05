<template>
  <div class="pdf-block-wrapper">
    <a :href="src" target="_blank" rel="noopener noreferrer" class="pdf-card">
      <!-- Icono PDF Decorativo -->
      <div class="pdf-icon-box">
        <q-icon name="picture_as_pdf" size="32px" color="red-5" />
      </div>

      <!-- Información del Recurso -->
      <div class="pdf-info">
        <div class="pdf-title">{{ title }}</div>
        <div class="pdf-meta">
          <span class="meta-tag">PDF</span>
          <span v-if="size" class="meta-separator">•</span>
          <span v-if="size">{{ size }}</span>
        </div>
        <div v-if="description" class="pdf-description">
          {{ description }}
        </div>
      </div>

      <!-- Icono de Acción (Flecha) -->
      <div class="pdf-action">
        <q-icon name="open_in_new" size="20px" />
      </div>
    </a>
  </div>
</template>

<script setup lang="ts">
// No necesitamos imports complejos, solo props limpias
defineProps<{
  src: string;
  title: string; // Reemplaza a 'label' para ser más semántico
  description?: string; // Opcional: para dar contexto (ej: "Datasheet v2.0")
  size?: string; // Opcional: ej: "2.4 MB"
}>();
</script>

<style scoped>
.pdf-block-wrapper {
  width: 100%;
  margin-bottom: 1.5rem;
}

/* TARJETA PRINCIPAL */
.pdf-card {
  display: flex;
  align-items: center;
  gap: 16px;
  padding: 16px;
  border-radius: 12px;
  background: rgba(30, 41, 59, 0.4); /* Slate-800 muy transparente */
  border: 1px solid rgba(148, 163, 184, 0.15); /* Borde sutil */
  text-decoration: none;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  backdrop-filter: blur(8px);
  position: relative;
  overflow: hidden;
}

/* Efecto Hover: Elevar y brillar */
.pdf-card:hover {
  background: rgba(30, 41, 59, 0.6);
  border-color: rgba(148, 163, 184, 0.3);
  transform: translateY(-2px);
  box-shadow: 0 10px 20px -5px rgba(0, 0, 0, 0.3);
}

/* ICONO PDF (Caja izquierda) */
.pdf-icon-box {
  flex-shrink: 0;
  width: 56px;
  height: 56px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: rgba(239, 68, 68, 0.1); /* Red-500 suave */
  border-radius: 10px;
  transition: background 0.3s;
}

.pdf-card:hover .pdf-icon-box {
  background: rgba(239, 68, 68, 0.15);
}

/* INFO CENTRAL */
.pdf-info {
  flex: 1;
  min-width: 0; /* Para que el text-overflow funcione */
}

.pdf-title {
  color: #f1f5f9; /* Slate-100 */
  font-weight: 600;
  font-size: 1rem;
  margin-bottom: 4px;
  line-height: 1.3;
}

.pdf-meta {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 0.75rem;
  color: #94a3b8; /* Slate-400 */
  font-weight: 500;
}

.meta-tag {
  background: rgba(148, 163, 184, 0.1);
  padding: 2px 6px;
  border-radius: 4px;
  font-weight: 600;
  letter-spacing: 0.5px;
}

.meta-separator {
  opacity: 0.5;
}

.pdf-description {
  margin-top: 6px;
  font-size: 0.85rem;
  color: #cbd5e1; /* Slate-300 */
  line-height: 1.4;
  display: -webkit-box;
  -webkit-line-clamp: 2; /* Limitar a 2 líneas */
  -webkit-box-orient: vertical;
  overflow: hidden;
}

/* ACCIÓN (Flecha derecha) */
.pdf-action {
  flex-shrink: 0;
  color: #64748b; /* Slate-500 */
  transition:
    transform 0.3s,
    color 0.3s;
}

.pdf-card:hover .pdf-action {
  color: #38bdf8; /* Sky-400 */
  transform: translateX(4px) rotate(-45deg); /* Efecto "salir" */
}

/* RESPONSIVE */
@media (max-width: 600px) {
  .pdf-card {
    padding: 12px;
    gap: 12px;
  }

  .pdf-icon-box {
    width: 48px;
    height: 48px;
  }

  .pdf-title {
    font-size: 0.95rem;
  }
}
</style>
