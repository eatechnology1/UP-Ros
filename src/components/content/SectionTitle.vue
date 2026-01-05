<template>
  <h2 class="section-title">
    <slot />
  </h2>
</template>

<style scoped>
.section-title {
  width: 100%;
  margin: 0 0 24px 0; /* Margen inferior para separar del contenido */
  padding-bottom: 12px;

  /* Tipografía */
  font-size: 1.75rem;
  font-weight: 700;
  color: #f1f5f9; /* Slate-100 */
  line-height: 1.2;
  letter-spacing: -0.02em;

  /* Línea base sutil */
  border-bottom: 1px solid rgba(148, 163, 184, 0.15);
  position: relative;
}

/* Barra de acento (El toque "Tech") */
.section-title::after {
  content: '';
  position: absolute;
  bottom: -1px; /* Se solapa con el borde base */
  left: 0;
  width: 60px; /* Longitud de la barra de acento */
  height: 3px; /* Grosor */
  background: linear-gradient(90deg, #38bdf8, #818cf8); /* Cyan-400 a Indigo-400 */
  border-radius: 2px;
  box-shadow: 0 1px 2px rgba(56, 189, 248, 0.3); /* Sutil brillo */
}

/* RESPONSIVE */
@media (max-width: 768px) {
  .section-title {
    font-size: 1.5rem; /* Un poco más pequeño en móviles */
    margin-bottom: 20px;
    padding-bottom: 10px;
  }
}
</style>
