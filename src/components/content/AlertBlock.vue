<template>
  <div class="alert-block" :class="type" role="alert">
    <div class="alert-icon">
      <q-icon :name="icon" :color="iconColor" size="20px" />
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
    />
  </div>
</template>

<script setup lang="ts">
import { computed } from 'vue';

interface AlertConfig {
  icon: string;
  iconColor: string;
  bg: string;
  border: string;
  text: string;
}

const props = withDefaults(
  defineProps<{
    title?: string;
    type?: 'info' | 'success' | 'warning' | 'danger';
    dismissible?: boolean;
  }>(),
  {
    type: 'info',
    dismissible: false,
  },
);

const alertConfig = computed<AlertConfig>(
  () =>
    ({
      info: {
        icon: 'info',
        iconColor: '#2196f3',
        bg: 'rgba(33, 150, 243, 0.12)',
        border: 'rgba(33, 150, 243, 0.3)',
        text: '#e6edf7',
      },
      success: {
        icon: 'check_circle',
        iconColor: '#4caf50',
        bg: 'rgba(76, 175, 80, 0.12)',
        border: 'rgba(76, 175, 80, 0.3)',
        text: '#dcfce7',
      },
      warning: {
        icon: 'warning',
        iconColor: '#ff9800',
        bg: 'rgba(255, 152, 0, 0.12)',
        border: 'rgba(255, 152, 0, 0.3)',
        text: '#fef3c7',
      },
      danger: {
        icon: 'error',
        iconColor: '#f44336',
        bg: 'rgba(244, 67, 54, 0.12)',
        border: 'rgba(244, 67, 54, 0.3)',
        text: '#fee2e2',
      },
    })[props.type ?? 'info'] || {
      icon: 'info',
      iconColor: '#2196f3',
      bg: 'rgba(33, 150, 243, 0.12)',
      border: 'rgba(33, 150, 243, 0.3)',
      text: '#e6edf7',
    },
);

const icon = computed(() => alertConfig.value.icon);
const iconColor = computed(() => alertConfig.value.iconColor);

function handleDismiss() {
  console.log('Alert dismissed:', props.title);
}
</script>

<style scoped>
.alert-block {
  max-width: 900px;
  margin: 32px auto;
  display: flex;
  align-items: flex-start;
  gap: 16px;
  padding: 24px;
  border-radius: 16px;
  backdrop-filter: blur(16px);
  -webkit-backdrop-filter: blur(16px);
  border: 1px solid transparent;
  transition: all 0.25s ease;
  position: relative;
  overflow: hidden;
}

.alert-block::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  height: 4px;
  background: linear-gradient(90deg, currentColor, transparent);
}

.alert-block:hover {
  transform: translateY(-2px);
  box-shadow: 0 20px 60px rgba(15, 23, 42, 0.8);
}

.alert-icon {
  flex-shrink: 0;
  margin-top: 2px;
  opacity: 0.9;
}

.alert-content {
  flex: 1;
}

.alert-title {
  color: #e6edf7;
  font-size: 1.1rem;
  font-weight: 600;
  margin-bottom: 4px;
  letter-spacing: 0.3px;
}

.alert-body {
  color: #cbd5f5;
  line-height: 1.6;
  font-size: 1rem;
}

.alert-dismiss {
  opacity: 0.7;
  margin-left: auto !important;
}

.alert-dismiss:hover {
  opacity: 1;
}

/* INFO */
.info {
  background: rgba(33, 150, 243, 0.12);
  border-color: rgba(33, 150, 243, 0.3);
}

.info::before {
  background-color: #2196f3;
}

/* SUCCESS */
.success {
  background: rgba(76, 175, 80, 0.12);
  border-color: rgba(76, 175, 80, 0.3);
}

.success::before {
  background-color: #4caf50;
}

/* WARNING */
.warning {
  background: rgba(255, 152, 0, 0.12);
  border-color: rgba(255, 152, 0, 0.3);
}

.warning::before {
  background-color: #ff9800;
}

/* DANGER */
.danger {
  background: rgba(244, 67, 54, 0.12);
  border-color: rgba(244, 67, 54, 0.3);
}

.danger::before {
  background-color: #f44336;
}

@media (max-width: 768px) {
  .alert-block {
    margin: 24px 16px;
    padding: 20px;
    flex-direction: column;
    gap: 12px;
  }

  .alert-dismiss {
    position: absolute !important;
    top: 12px;
    right: 12px;
  }
}
</style>
