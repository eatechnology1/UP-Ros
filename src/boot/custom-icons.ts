// src/boot/custom-icons.ts
import { boot } from 'quasar/wrappers';

// --- IMPORTACIÓN MASIVA DE REMIX ICONS ---
import {
  // 1. Flechas y Navegación
  remArrowUpSLine,
  remArrowRightSLine,
  remArrowDownSLine,
  remArrowLeftSLine,
  remArrowDropDownFill,
  remSkipBackLine,
  remSkipForwardLine,

  // 2. Estados y Alertas
  remCheckboxCircleFill,
  remErrorWarningFill,
  remInformationFill,
  remAlertFill,

  // 3. Edición y UI General
  remCloseLine,
  remCloseCircleFill,
  remAddLine,
  remCheckLine,
  remMenuLine,
  remMore2Fill,

  // 4. Componentes Específicos
  remUploadCloud2Line,
  remCloseCircleLine,
  remCalendarLine,
  remStarFill,
  // (Aquí borramos remArrowUpDownLine porque no se estaba usando)
} from 'quasar-extras-svg-icons/remix-icons';

export default boot(({ app }) => {
  const iconSet = app.config.globalProperties.$q.iconSet;

  if (iconSet) {
    // 1. TIPOS
    iconSet.type.positive = remCheckboxCircleFill;
    iconSet.type.negative = remErrorWarningFill;
    iconSet.type.info = remInformationFill;
    iconSet.type.warning = remAlertFill;

    // 2. FLECHAS
    iconSet.arrow.up = remArrowUpSLine;
    iconSet.arrow.right = remArrowRightSLine;
    iconSet.arrow.down = remArrowDownSLine;
    iconSet.arrow.left = remArrowLeftSLine;
    iconSet.arrow.dropdown = remArrowDropDownFill;

    // 3. PAGINACIÓN
    iconSet.pagination.first = remSkipBackLine;
    iconSet.pagination.prev = remArrowLeftSLine;
    iconSet.pagination.next = remArrowRightSLine;
    iconSet.pagination.last = remSkipForwardLine;

    // 4. TABLAS
    iconSet.table.arrowUp = remArrowUpSLine;
    iconSet.table.warning = remAlertFill;
    iconSet.table.firstPage = remSkipBackLine;
    iconSet.table.prevPage = remArrowLeftSLine;
    iconSet.table.nextPage = remArrowRightSLine;
    iconSet.table.lastPage = remSkipForwardLine;

    // 5. FORMULARIOS
    iconSet.field.clear = remCloseCircleLine;
    iconSet.field.error = remErrorWarningFill;
    iconSet.chip.remove = remCloseCircleFill;
    iconSet.chip.selected = remCheckLine;

    // 6. OTROS
    iconSet.expansionItem.icon = remArrowDownSLine;
    iconSet.expansionItem.denseIcon = remArrowDownSLine;

    iconSet.uploader.done = remCheckLine;
    iconSet.uploader.clear = remCloseLine;
    iconSet.uploader.add = remAddLine;
    iconSet.uploader.upload = remUploadCloud2Line;
    iconSet.uploader.removeQueue = remMenuLine;
    iconSet.uploader.removeUploaded = remCloseLine;

    iconSet.datetime.arrowLeft = remArrowLeftSLine;
    iconSet.datetime.arrowRight = remArrowRightSLine;
    iconSet.datetime.now = remCheckLine;
    iconSet.datetime.today = remCalendarLine;

    iconSet.rating.icon = remStarFill;

    iconSet.stepper.done = remCheckLine;
    iconSet.stepper.active = remMore2Fill;
    iconSet.stepper.error = remErrorWarningFill;
  }
});
