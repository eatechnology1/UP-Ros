<template>
  <div v-if="isVisible" class="alert-block" :class="type" role="alert">
    <div class="alert-icon">
      <!-- Usamos style para color exacto -->
      <q-icon :name="icon" :style="{ color: iconColor }" size="24px" />
    </div>

    <div class="alert-content">
      <div class="alert-title" v-if="title">{{ title }}</div>
      <div class="alert-body">
        <slot />
      </div>
    </div>

    <q-btn
      v-if="dismissible"
      icon="close"
      flat
      dense
      round
      size="sm"
      class="alert-dismiss"
      @click="handleDismiss"
      aria-label="Cerrar alerta"
    />
  </div>
</template>

<script setup lang="ts">
import { computed, ref } from 'vue';

// 1. Definición de Tipos
interface AlertConfig {
  icon: string;
  iconColor: string;
}

type AlertType = 'info' | 'success' | 'warning' | 'danger';

// 2. Props
const props = withDefaults(
  defineProps<{
    title?: string;
    type?: AlertType;
    dismissible?: boolean;
  }>(),
  {
    type: 'info',
    dismissible: false,
  },
);

// 3. Emits y Estado
const emit = defineEmits<{
  (e: 'dismiss'): void;
}>();

const isVisible = ref(true);

// 4. Configuración Estática (Tipada estrictamente)
const configs: Record<AlertType, AlertConfig> = {
  info: { icon: 'info', iconColor: '#2196f3' },
  success: { icon: 'check_circle', iconColor: '#4caf50' },
  warning: { icon: 'warning', iconColor: '#ff9800' },
  danger: { icon: 'error', iconColor: '#f44336' },
};

// 5. Computed (CORREGIDO)
// El operador ?? asegura que nunca se retorne undefined, satisfaciendo a TypeScript
const alertConfig = computed<AlertConfig>(() => {
  return configs[props.type] ?? configs.info;
});

const icon = computed(() => alertConfig.value.icon);
const iconColor = computed(() => alertConfig.value.iconColor);

function handleDismiss() {
  isVisible.value = false;
  emit('dismiss');
}
</script>

<style scoped>
.alert-block {
  width: 100%;
  display: flex;
  align-items: flex-start;
  gap: 16px;
  padding: 20px 24px;
  border-radius: 12px;
  backdrop-filter: blur(12px);
  -webkit-backdrop-filter: blur(12px);
  border: 1px solid transparent;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  position: relative;
  overflow: hidden;
}

/* Barra superior de acento */
.alert-block::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  height: 3px;
  opacity: 0.8;
}

.alert-block:hover {
  transform: translateY(-2px);
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.2);
}

.alert-icon {
  flex-shrink: 0;
  margin-top: 2px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: rgba(255, 255, 255, 0.05);
  border-radius: 50%;
  padding: 8px;
}

.alert-content {
  flex: 1;
  min-width: 0;
}

.alert-title {
  color: #ffffff;
  font-size: 1.05rem;
  font-weight: 700;
  margin-bottom: 6px;
  letter-spacing: 0.3px;
  line-height: 1.3;
}

.alert-body {
  color: #cbd5f5;
  line-height: 1.6;
  font-size: 0.95rem;
}

.alert-body :deep(a) {
  color: inherit;
  text-decoration: underline;
  text-underline-offset: 2px;
}

.alert-dismiss {
  opacity: 0.6;
  margin-left: 8px !important;
  color: #cbd5f5;
  transition: opacity 0.2s;
}

.alert-dismiss:hover {
  opacity: 1;
  background: rgba(255, 255, 255, 0.1);
}

/* VARIANTES */

/* INFO */
.info {
  background: rgba(33, 150, 243, 0.12);
  border-color: rgba(33, 150, 243, 0.25);
}
.info::before {
  background-color: #2196f3;
}
.info .alert-title {
  color: #90caf9;
}

/* SUCCESS */
.success {
  background: rgba(76, 175, 80, 0.12);
  border-color: rgba(76, 175, 80, 0.25);
}
.success::before {
  background-color: #4caf50;
}
.success .alert-title {
  color: #a5d6a7;
}

/* WARNING */
.warning {
  background: rgba(255, 152, 0, 0.12);
  border-color: rgba(255, 152, 0, 0.25);
}
.warning::before {
  background-color: #ff9800;
}
.warning .alert-title {
  color: #ffcc80;
}

/* DANGER */
.danger {
  background: rgba(244, 67, 54, 0.12);
  border-color: rgba(244, 67, 54, 0.25);
}
.danger::before {
  background-color: #f44336;
}
.danger .alert-title {
  color: #ef9a9a;
}

/* RESPONSIVE */
@media (max-width: 600px) {
  .alert-block {
    padding: 16px;
    gap: 12px;
  }

  .alert-icon {
    padding: 6px;
  }

  .alert-icon .q-icon {
    font-size: 20px !important;
  }

  .alert-dismiss {
    position: absolute;
    top: 8px;
    right: 8px;
  }
}
</style>
