import type { SectionContent } from '../types';

export const intro: SectionContent = {
  id: 'intro',
  title: 'Introducción a ROS 2',
  markdown: `
## ¿Qué es ROS 2?

ROS 2 (Robot Operating System 2) es un framework de software orientado al desarrollo de aplicaciones robóticas distribuidas.

Está diseñado para sistemas modernos, soportando comunicación en tiempo real, seguridad y escalabilidad.

## ¿Por qué ROS 2?

- Arquitectura distribuida
- Comunicación basada en DDS
- Compatibilidad multiplataforma
- Uso extensivo en investigación e industria

Este material tiene como objetivo proporcionar una base sólida para el aprendizaje de ROS 2 desde una perspectiva mecatrónica.
  `,
  images: ['ros2_architecture.png'],
};
