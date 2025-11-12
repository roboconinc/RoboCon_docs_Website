import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for the Implementation docs plugin
 * This file is used by the implementation plugin which serves content from the implementation/ directory
 */

const sidebars: SidebarsConfig = {
  // Hardware Sidebar (Implementation plugin)
  hardwareSidebar: [
    {
      type: 'doc',
      id: 'hardware/index',
      label: 'Hardware Overview',
    },
    {
      type: 'category',
      label: 'Arm',
          items: [
        {
          type: 'doc',
          id: 'hardware/arm/index',
        },
        {
          type: 'doc',
          id: 'hardware/arm/ae-30-cobot',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-example',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-finger-2-gripper-30kg-on-30kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-finger-2-gripper-5kg-on-10kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-finger-2-gripper-5kg-on-5kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-finger-5-gripper-10kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-cobot-05kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-cobot-05kg-branding',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-cobot-15kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-cobot-15kg-branding',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-cobot-30kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-industrial-10kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-industrial-30kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-industrial-50kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-industrial-50kg-base',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-industrial-50kg-branding',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-industrial-50kg-manual',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-industrial-60kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/arm-robocon-industrial-long-30kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirbr2260a',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirse1506a',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirus0805a',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirus0805a-media',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirus1510av11',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirus2030av11',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirus2110a',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirus2550a',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirus2550a-media',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirus2550a-proposal',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirus3030a',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirxz0805a',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirxz0805a-proposal',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirxz1515a',
        },
        {
          type: 'doc',
          id: 'hardware/arm/fairino',
        },
        {
          type: 'doc',
          id: 'hardware/arm/fairino-fr10',
        },
        {
          type: 'doc',
          id: 'hardware/arm/fairino-fr30',
        },
        {
          type: 'doc',
          id: 'hardware/arm/fairino-fr5',
        },
        {
          type: 'doc',
          id: 'hardware/arm/gearbox-j1-rv-200c-142',
        },
        {
          type: 'doc',
          id: 'hardware/arm/gearbox-j2-rv-320e-171',
        },
        {
          type: 'doc',
          id: 'hardware/arm/gearbox-j3-rv-160e-171',
        },
        {
          type: 'doc',
          id: 'hardware/arm/gearbox-j4-rv-50c-110',
        },
        {
          type: 'doc',
          id: 'hardware/arm/gearbox-j5-rv-40e-121',
        },
        {
          type: 'doc',
          id: 'hardware/arm/gearbox-j6-rv-20e-121',
        },
        {
          type: 'doc',
          id: 'hardware/arm/kuka',
        },
        {
          type: 'doc',
          id: 'hardware/arm/kuka-kr70r2100',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-borunte-brtirxz0805a-j1-48vdc',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-emda020h1jd3-220v-200w',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-emda040h1jd3-220v-400w',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-emda100h1jd3-220v-1000w',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-emda100h2jd0-380v-1000w',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-emda170h3jd1-220v-1700w',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-emda200h1jd3-220v-1800w',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-emda200h2jd0-380v-1800w',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-emda440h2jd0-380v-4400w',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-emda750h2jd0-380v-7500w',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-sm-180a29c20c-mb34n',
        },
        {
          type: 'doc',
          id: 'hardware/arm/motor-sxd456',
        },
        {
          type: 'doc',
          id: 'hardware/arm/nkrt-650',
        },
        {
          type: 'doc',
          id: 'hardware/arm/szgh20241101',
        },
        {
          type: 'doc',
          id: 'hardware/arm/wuhan-maxwave-mw-1508',
        },
        {
          type: 'doc',
          id: 'hardware/arm/wuhan-maxwave-mw-1510',
        },
      ],
    },
    {
      type: 'category',
      label: 'Arm-Equipped',
          items: [
        {
          type: 'doc',
          id: 'hardware/arm-equipped/index',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/drywall-cutter',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/drywall-placer',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/gripper-05kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/gripper-15kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/gripper-50kg',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/mud',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/outdoor-placer',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/paint',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/palletizing',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/router',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/screw',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/sheathing-placer',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/tape',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/texture',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/welding',
        },
        {
          type: 'doc',
          id: 'hardware/arm-equipped/wiper',
        },
      ],
    },
    {
      type: 'category',
      label: 'Robots',
      items: [
        {
          type: 'doc',
          id: 'hardware/robots/index',
        },
        {
          type: 'doc',
          id: 'hardware/robots/mini-crane-tracked-3000kg-electric',
        },
        {
          type: 'doc',
          id: 'hardware/robots/mini-crane-tracked-3000kg-hydraulic',
        },
        {
          type: 'doc',
          id: 'hardware/robots/loader-tracked-300kg-hydraulic',
        },
        {
          type: 'doc',
          id: 'hardware/robots/excavator-tracked',
        },
        {
          type: 'doc',
          id: 'hardware/robots/demolisher-tracked',
        },
        {
          type: 'doc',
          id: 'hardware/robots/servicer-tracked-15kg-electric',
        },
        {
          type: 'doc',
          id: 'hardware/robots/servicer-wheeled-15kg-electric',
        },
        {
          type: 'doc',
          id: 'hardware/robots/sheather-tracked-50kg-electric',
        },
        {
          type: 'doc',
          id: 'hardware/robots/sheather-wheeled-50kg-electric',
        },
        {
          type: 'doc',
          id: 'hardware/robots/transporter-tracked-1000kg',
        },
        {
          type: 'doc',
          id: 'hardware/robots/transporter-wheeled-1000kg',
        },
        {
          type: 'doc',
          id: 'hardware/robots/rotary-workstation',
        },
      ],
    },
    {
      type: 'category',
      label: 'Mobility',
          items: [
        {
          type: 'doc',
          id: 'hardware/mobility/index',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/amr',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/amr-borunte-brtagv21050a',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_3f',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_csg-d80-012-20',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_d80-09-20',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_d80-80-20',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_ecc4t242840brgbr',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_jbl160-l2-12-22-55-110-145-m8',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_rv-80em',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_rv-80em-101',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_rv-reducer-220bx-80e',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_suregear-pgd110-50a2',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_vrt-110-10-p1',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/gearbox_zk-rotary-platform',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/motor_golden-120vdc-10kw',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/motor_golden-144vdc-20kw',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/motor_golden-48vdc-3kw',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/motor_kqw-23dc-24vdc-400w',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/motor_mdw114-2-22-1',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/motor_xq-085t-24vdc-850w',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/motor-driver_ez-a48400',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/motor-driver_zapi-dualpmxhp',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/tracked-undercarriage_144vdc-20kw',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/tracked-undercarriage_48vdc-3kw',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/tracks_gaoshan',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/tracks_gaoshan-mechanical-gsst',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/tracks_hebei-jinliang-jlh04',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/tracks_hebei-jinliang-jlk20',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/tracks_hebei-jinliang-jlz04',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/wheel_100-x-323',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/wheel_wheel-12x4-5',
        },
        {
          type: 'doc',
          id: 'hardware/mobility/wheel_xzl-200-1500-300',
        },
      ],
    },
    {
      type: 'category',
      label: 'Electric',
          items: [
        {
          type: 'doc',
          id: 'hardware/electric/index',
        },
        {
          type: 'doc',
          id: 'hardware/electric/battery-duralast-lead-12v-60ah',
        },
        {
          type: 'doc',
          id: 'hardware/electric/battery-duralast-platinum-agm',
        },
        {
          type: 'doc',
          id: 'hardware/electric/battery-eco-worthy-lifepo4-48v-50ah',
        },
        {
          type: 'doc',
          id: 'hardware/electric/battery-lfp-72v-300ah',
        },
        {
          type: 'doc',
          id: 'hardware/electric/battery-lipuls-lifepo4-12.8v-100ah',
        },
        {
          type: 'doc',
          id: 'hardware/electric/battery-lossigy-lifepo4-48v-100ah',
        },
        {
          type: 'doc',
          id: 'hardware/electric/battery-maxwell-durablue-24v-375f',
        },
        {
          type: 'doc',
          id: 'hardware/electric/battery-scremower-lifepo4-48v-100ah',
        },
        {
          type: 'doc',
          id: 'hardware/electric/battery-tesla-catl-lfp-pack-btf0',
        },
        {
          type: 'doc',
          id: 'hardware/electric/battery-tesla-lfp-model-3-2021',
        },
        {
          type: 'doc',
          id: 'hardware/electric/cable-drag-chain-10x20mm',
        },
        {
          type: 'doc',
          id: 'hardware/electric/cable-drag-chain-15x40mm',
        },
        {
          type: 'doc',
          id: 'hardware/electric/charger-epc602-4840-ep-01a',
        },
        {
          type: 'doc',
          id: 'hardware/electric/charger-gwhc3-3.3kw',
        },
        {
          type: 'doc',
          id: 'hardware/electric/charger-yewsun-6600m-80v-6kw-air',
        },
        {
          type: 'doc',
          id: 'hardware/electric/contactor-bsbc7-150-150a-12v-coil',
        },
        {
          type: 'doc',
          id: 'hardware/electric/converter-mean-well-ddr-120c-12',
        },
        {
          type: 'doc',
          id: 'hardware/electric/converter-mean-well-ddr-480c-24',
        },
        {
          type: 'doc',
          id: 'hardware/electric/converter-mean-well-ddr-60l-5',
        },
        {
          type: 'doc',
          id: 'hardware/electric/converter-mean-well-sd-1000l-24',
        },
        {
          type: 'doc',
          id: 'hardware/electric/converter-ystech-ibdc10050',
        },
        {
          type: 'doc',
          id: 'hardware/electric/converter-ystech-ibdc30060',
        },
        {
          type: 'doc',
          id: 'hardware/electric/inlet-bosslyn-ss2-50-inlet-male-short',
        },
        {
          type: 'doc',
          id: 'hardware/electric/inlet-leviton-inlet-receptacle-125v',
        },
        {
          type: 'doc',
          id: 'hardware/electric/inlet-leviton-nema-5-15p-inlet-male',
        },
        {
          type: 'doc',
          id: 'hardware/electric/inlet-ss2-50-inlet-male-long',
        },
        {
          type: 'doc',
          id: 'hardware/electric/inverter-junbpaw-lge-5080',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-aryar-rj45-coupler-din',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-brake-resistor-300w',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-circuit-breaker-300a',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-phoenix-cable-duct-30x60',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-phoenix-cd-30x60-cable-duct-30mm',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-sensata-crydom-m5060sb1200',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-sh-zl3-brake-rectifier',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-uk2.5b-terminal-01pin',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-uk2.5b-terminal-15pin',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-uk2.5b-terminal-20pin',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-uk2.5b-terminal-25pin',
        },
        {
          type: 'doc',
          id: 'hardware/electric/part-zoerax-rj45-coupler-din',
        },
        {
          type: 'doc',
          id: 'hardware/electric/power-block-daiertek-12v-300a',
        },
        {
          type: 'doc',
          id: 'hardware/electric/power-block-rvboatpat-12v-150a',
        },
        {
          type: 'doc',
          id: 'hardware/electric/psu-rgeek-xl-300w-19v',
        },
        {
          type: 'doc',
          id: 'hardware/electric/resistor-brake-resistor-300w',
        },
      ],
    },
    {
      type: 'category',
      label: 'Compute',
          items: [
        {
          type: 'doc',
          id: 'hardware/compute/index',
        },
        {
          type: 'category',
          label: 'Camera',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/camera/jec-zn2133',
            },
          ],
        },
        {
          type: 'category',
          label: 'Case',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/case/lenovo-tab-p12',
            },
          ],
        },
        {
          type: 'category',
          label: 'Computer',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/computer/esp32-ch340c',
            },
            {
              type: 'doc',
              id: 'hardware/compute/computer/lcd-rasberry-pi-5-can-hat',
            },
            {
              type: 'doc',
              id: 'hardware/compute/computer/raspberry-pi-5',
            },
            {
              type: 'doc',
              id: 'hardware/compute/computer/waveshare-isolated-can-bus',
            },
            {
              type: 'doc',
              id: 'hardware/compute/computer/waveshare-rs485-can-hat',
            },
          ],
        },
        {
          type: 'category',
          label: 'Controller',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/controller/cyber-mi-cfb108',
            },
            {
              type: 'doc',
              id: 'hardware/compute/controller/jpf4816-fan-speed',
            },
            {
              type: 'doc',
              id: 'hardware/compute/controller/led-bc-204',
            },
            {
              type: 'doc',
              id: 'hardware/compute/controller/signal-receiver',
            },
          ],
        },
        {
          type: 'category',
          label: 'CPU',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/cpu/amd-wraith-stealth-socket-am4',
            },
          ],
        },
        {
          type: 'category',
          label: 'Depth Camera',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/depth-camera/ds800-e1',
            },
            {
              type: 'doc',
              id: 'hardware/compute/depth-camera/oak-d-s2',
            },
            {
              type: 'doc',
              id: 'hardware/compute/depth-camera/tm815-ix-e1',
            },
            {
              type: 'doc',
              id: 'hardware/compute/depth-camera/tm851-e1',
            },
          ],
        },
        {
          type: 'category',
          label: 'Energy Meter',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/energy-meter/acrel-ahkc-uka',
            },
            {
              type: 'doc',
              id: 'hardware/compute/energy-meter/acrel-amc16-dett',
            },
            {
              type: 'doc',
              id: 'hardware/compute/energy-meter/acrel-amc16z',
            },
          ],
        },
        {
          type: 'category',
          label: 'Fan',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/fan/computer-fan-120mm',
            },
            {
              type: 'doc',
              id: 'hardware/compute/fan/raspberry-pi-active-cooler',
            },
          ],
        },
        {
          type: 'category',
          label: 'GPU',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/gpu/gigabyte-radeon-rx-7600-xt',
            },
            {
              type: 'doc',
              id: 'hardware/compute/gpu/xfx-merc310-amd-radeon-rx-7900xtx',
            },
          ],
        },
        {
          type: 'category',
          label: 'LCD',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/lcd/ips-touchscreen-lcd-7-inch',
            },
          ],
        },
        {
          type: 'category',
          label: 'LED',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/led/ws2811-dc24v-720-white-pcb',
            },
          ],
        },
        {
          type: 'category',
          label: 'Lidar',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/lidar/benewake-tf03-180',
            },
            {
              type: 'doc',
              id: 'hardware/compute/lidar/hinson-de-4211',
            },
            {
              type: 'doc',
              id: 'hardware/compute/lidar/hinson-de-4511',
            },
            {
              type: 'doc',
              id: 'hardware/compute/lidar/hinson-se-1035',
            },
            {
              type: 'doc',
              id: 'hardware/compute/lidar/unitree-4d-lidar-l2',
            },
          ],
        },
        {
          type: 'category',
          label: 'Motherboard',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/motherboard/asrock-b450m-hdv-r4.0-am4',
            },
          ],
        },
        {
          type: 'category',
          label: 'Mount',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/mount/kksb-din-rail-rasberry-pi',
            },
          ],
        },
        {
          type: 'category',
          label: 'Pan/Tilt',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/pantilt/jec-j-pt-760',
            },
          ],
        },
        {
          type: 'category',
          label: 'Part',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/part/gperhuan-2u-micro-atx-compact',
            },
          ],
        },
        {
          type: 'category',
          label: 'PCIe',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/pcie/advantech-pcie-1680-b',
            },
            {
              type: 'doc',
              id: 'hardware/compute/pcie/crucial-p310-1tb-2280-pcie',
            },
            {
              type: 'doc',
              id: 'hardware/compute/pcie/linkstek-pcie-n600',
            },
            {
              type: 'doc',
              id: 'hardware/compute/pcie/tp-link-archer-tx55e-ax3000',
            },
          ],
        },
        {
          type: 'category',
          label: 'Platform',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/platform/sp0503-wxr',
            },
          ],
        },
        {
          type: 'category',
          label: 'PLC',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/plc/10ioa08-8di-8do-24v',
            },
            {
              type: 'doc',
              id: 'hardware/compute/plc/10ioa12-12di-12do-24v',
            },
            {
              type: 'doc',
              id: 'hardware/compute/plc/2ao-8ai-8di-8do-24v',
            },
            {
              type: 'doc',
              id: 'hardware/compute/plc/fx3u-24mt',
            },
            {
              type: 'doc',
              id: 'hardware/compute/plc/logo-6ed1052-2cc08-0ba2',
            },
            {
              type: 'doc',
              id: 'hardware/compute/plc/r4ivb02-power-6-25v-in-0-10v',
            },
          ],
        },
        {
          type: 'category',
          label: 'RAM',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/ram/corsair-vengeance-lpx-ddr4-ram-32gb',
            },
          ],
        },
        {
          type: 'category',
          label: 'Relay',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/relay/deca-mlr-108',
            },
            {
              type: 'doc',
              id: 'hardware/compute/relay/relay-module-4ch-rs485',
            },
            {
              type: 'doc',
              id: 'hardware/compute/relay/xinlihui-n4roc04-24v',
            },
          ],
        },
        {
          type: 'category',
          label: 'Remote',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/remote/cyber-mi-cfb108',
            },
          ],
        },
        {
          type: 'category',
          label: 'Sensor',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/sensor/bw-imu50',
            },
            {
              type: 'doc',
              id: 'hardware/compute/sensor/bw-mins57-rs485',
            },
            {
              type: 'doc',
              id: 'hardware/compute/sensor/bwsensing-sec295',
            },
            {
              type: 'doc',
              id: 'hardware/compute/sensor/sensepa-tilt-3',
            },
          ],
        },
        {
          type: 'category',
          label: 'Switch',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/switch/trendnet-din-switch-10-gbps',
            },
          ],
        },
        {
          type: 'category',
          label: 'Tablet',
          items: [
            {
              type: 'doc',
              id: 'hardware/compute/tablet/lenovo-tab-p12',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Ship',
          items: [
        {
          type: 'doc',
          id: 'hardware/ship/index',
        },
        {
          type: 'doc',
          id: 'hardware/ship/ship-arleigh-burke-destroyer',
        },
        {
          type: 'doc',
          id: 'hardware/ship/ship-unit',
        },
      ],
    },
    {
      type: 'category',
      label: 'Cabinet',
          items: [
        {
          type: 'doc',
          id: 'hardware/cabinet/index',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/cabinet-hanging-sensor',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/cabinet-il30-back',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/cabinet-il30-back-tall',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/cabinet-il30-front',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/drawer-il30-arm',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/drawer-il30-base-control',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/drawer-il30-battery',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/drawer-il30-boot',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/drawer-il30-computer',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/drawer-rack17-computer',
        },
        {
          type: 'doc',
          id: 'hardware/cabinet/drawer-rack17-power',
        },
      ],
    },
    {
      type: 'category',
      label: 'MEWP',
      items: [
        {
          type: 'doc',
          id: 'hardware/mewp/index',
        },
        {
          type: 'category',
          label: 'Boom',
          items: [
            {
              type: 'doc',
              id: 'hardware/mewp/boom/index',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/boom/MEWP_Boom_JLG X600AJ',
            },
          ],
        },
        {
          type: 'category',
          label: 'Crane',
          items: [
            {
              type: 'doc',
              id: 'hardware/mewp/crane/index',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_GBT-8',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Jekko SPX532',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Jinan Gradin Machinery Co., Ltd',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Jinan PLK CL3.0',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Jinan Plk Machinery Co., Ltd',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Jining Hongrun',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Maeda MC104C',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Maeda MC285C-3',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Qingdao Zhongmaotong Machinery Co., Ltd',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Shandong Belift Machinery Co.,ltd',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Shandong Cathay Machinery Co., Ltd',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Shandong Rich Yuanda',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_Shandong Yuanxing',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/crane/MEWP_Crane_SPYDERCRANE URW295_similar',
            },
          ],
        },
        {
          type: 'category',
          label: 'Mast',
          items: [
            {
              type: 'doc',
              id: 'hardware/mewp/mast/index',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/mast/MEWP_Mast_Star 6 CRAWLER',
            },
          ],
        },
        {
          type: 'category',
          label: 'Scissor',
          items: [
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/index',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Athena 850',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Dingli JCPT0607DCH',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Dingli JCPT0608DCH',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Dingli JCPT0807DCH',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Genie GS-1530',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Genie GS-1930',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Henan Crane',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Henan Crane GTJZD06',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Hered HC0607EA',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Jiangsu Everstar',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Jiangsu Everstar EPT036ZF',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Jiangsu Everstar EPT0406_Edit',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Jiangsu Everstar EPT0608SP',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Jining Lovin',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Jining Lovin GTJZ06',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_JLG 1932ES',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_JLG AE1932',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_JLG ES1530L',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_LTMG LTWP0406',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Shandong Huateng',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Shandong Huateng GTJZD04',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Shandong Huateng GTJZD06',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Xiamen Tder TDWP0608',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Xiamen Tder ZTPV0305',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Xuzhou',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Xuzhou EPT0406',
            },
            {
              type: 'doc',
              id: 'hardware/mewp/scissor/MEWP_Scissor_Xuzhou EPT0608SP',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Robots',
      items: [
        {
          type: 'category',
          label: 'Servicer WE15',
          items: [
            {
              type: 'doc',
              id: 'hardware/robot/servicer-we15/index',
            },
            {
              type: 'doc',
              id: 'hardware/robot/servicer-we15/2024-12-17-BRAD',
            },
            {
              type: 'doc',
              id: 'hardware/robot/servicer-we15/2024-12-17-crankmechanical',
            },
            {
              type: 'doc',
              id: 'hardware/robot/servicer-we15/2024-12-19-crankmechanical',
            },
          ],
        },
        {
          type: 'category',
          label: 'iSheather',
          items: [
          {
            type: 'doc',
            id: 'hardware/robot/isheather/index',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-03-06-BRAD',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-03-13-crankmechanical-MEDIA',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-03-14-crankmechanical',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-03-14-crankmechanical-EDIT',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-03-14-crankmechanical-MEDIA',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-03-18-crankmechanical-EDIT',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-03-21-crankmechanical',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-03-21-crankmechanical-MEDIA',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-03-29-crankmechanical',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-04-12-crankmechanical',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-04-12-crankmechanical-Media',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-04-21-crankmechanical',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-04-28-BRAD',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-05-20-josalexc',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-05-28-HeNan-Crane',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-06-03-crankmechanical',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-06-03-crankmechanical-Edits',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-06-12-crankmechanical-Edits',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-06-16-crankmechanical',
          },
          {
            type: 'doc',
            id: 'hardware/robot/isheather/2025-06-16-crankmechanical-Edits',
          },
          ],
        },
        {
          type: 'category',
          label: 'Mini Crane Tracked 3000kg Electric',
          items: [
          {
            type: 'doc',
            id: 'hardware/robot/mini-crane-tracked-3000kg-electric/index',
          },
          {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-electric/2025-06-03-gopalmandal-tri',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-electric/2025-06-03-gopalmandal-tri-Edits',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-electric/2025-06-10-gopalmandal-tri',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-electric/2025-06-16-gopalmandal-tri',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-electric/2025-06-16-gopalmandal-tri-Media',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-electric/2025-06-18-gopalmandal-tri',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-electric/2025-07-21-gopalmandal-tri-Renderings',
        },
      ],
    },
    {
      type: 'category',
      label: 'Mini Crane Tracked 3000kg Hydraulic',
          items: [
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/index',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-19-Media',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-21-Measure',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-21-ShanDong-YuanXing',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-22-gopalmandal-tri',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-23-ShanDong-YuanXing-Media',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-23-ShanDong-YuanXing-Photos',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-24-gopalmandal-tri-Edits',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-24-ShanDong-YuanXing',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-26-BRAD',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-26-BRAD-Edits',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-28-ShanDong-YuanXing',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-29-ShanDong-YuanXing',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-31-gopalmandal-tri',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-05-31-gopalmandal-tri-Edits',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-06-03-gopalmandal-tri',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-06-03-gopalmandal-tri-Edits',
        },
        {
          type: 'doc',
          id: 'hardware/robot/mini-crane-tracked-3000kg-hydraulic/2025-06-10-ShanDong-YuanXing',
        },
      ],
    },
    {
      type: 'category',
      label: 'Mini Demolisher',
          items: [
        {
          type: 'doc',
          id: 'hardware/robot/mini-demolisher/index',
        },
      ],
    },
    {
      type: 'category',
      label: 'Mini Excavator',
          items: [
        {
          type: 'doc',
          id: 'hardware/robot/mini-excavator/index',
        },
      ],
    },
    {
      type: 'category',
      label: 'Front Loader Tracked 300kg Hydraulic',
          items: [
        {
          type: 'doc',
          id: 'hardware/robot/front-loader-tracked-300kg-hydraulic/index',
        },
        {
          type: 'doc',
          id: 'hardware/robot/front-loader-tracked-300kg-hydraulic/2025-04-26-Ant-Cloud',
        },
        {
          type: 'doc',
          id: 'hardware/robot/front-loader-tracked-300kg-hydraulic/2025-06-06-Ant-Cloud',
        },
        {
          type: 'doc',
          id: 'hardware/robot/front-loader-tracked-300kg-hydraulic/2025-06-10-Ant-Cloud',
        },
        {
          type: 'doc',
          id: 'hardware/robot/front-loader-tracked-300kg-hydraulic/2025-06-19-Ant-Cloud',
        },
        {
          type: 'doc',
          id: 'hardware/robot/front-loader-tracked-300kg-hydraulic/2025-06-30-aneebahsstudio',
        },
        {
          type: 'doc',
          id: 'hardware/robot/front-loader-tracked-300kg-hydraulic/2025-07-07-Ant-Cloud',
        },
      ],
    },
    {
      type: 'category',
      label: 'oServicer',
          items: [
        {
          type: 'doc',
          id: 'hardware/robot/oservicer/index',
        },
        {
          type: 'doc',
          id: 'hardware/robot/oservicer/2025-05-20-zaaimvalley',
        },
        {
          type: 'doc',
          id: 'hardware/robot/oservicer/2025-06-04-zaaimvalley-Drawings',
        },
        {
          type: 'doc',
          id: 'hardware/robot/oservicer/2025-06-07-zaaimvalley',
        },
        {
          type: 'doc',
          id: 'hardware/robot/oservicer/2025-06-14-zaaimvalley',
        },
      ],
    },
    {
      type: 'category',
      label: 'oSheather',
          items: [
        {
          type: 'doc',
          id: 'hardware/robot/osheather/index',
        },
      ],
    },
    {
      type: 'category',
      label: 'Rotary Workstation',
          items: [
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/index',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-08-30-Patent',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-01-Patent',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-01-rehan-tahir92',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-03-Patent',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-04-aqsafatima333',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-04-muthugeetha',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-05-aqsafatima333',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-05-rehan-tahir92',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-07-aqsafatima333',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-09-muthugeetha',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-10-muthugeetha',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-11-aqsafatima333',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-12-aqsafatima333',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-15-aqsafatima333',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-15-aqsafatima333-Edits',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-17-aqsafatima333-Edits',
        },
        {
          type: 'doc',
          id: 'hardware/robot/rotary-workstation/2024-10-20-aqsafatima333',
        },
      ],
    },
    {
      type: 'category',
      label: 'Transporter',
          items: [
        {
          type: 'doc',
          id: 'hardware/robot/transporter/index',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2024-10-15-wasifnadeem279',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2024-10-18-wasifnadeem279',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2024-10-21-wasifnadeem279',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-03-29-BRAD',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-04-08-peter-studioz',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-04-08-peter-studioz-Edits',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-04-09-yusuffart01',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-04-09-yusuffart01-EDITS',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-04-09-yusuffart01-Media',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-04-23-crankmechanical',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-04-23-Frame',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-05-09-crankmechanical',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-05-24-crankmechanical',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-05-24-crankmechanical-Edits',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-06-02-crankmechanical',
        },
        {
          type: 'doc',
          id: 'hardware/robot/transporter/2025-06-03-crankmechanical',
        },
      ],
    },
    ],
    },
    {
      type: 'category',
      label: 'End Effector',
      items: [
        {
          type: 'doc',
          id: 'hardware/end-effector/index',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/air-tank-yc-1l-csh',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/connector-bilian-bln-svk640-3r18-h20',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/connector-botu-tm815-ix-e1',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/connector-finger-2-gripper-5kg-on-10kg',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/connector-finger-2-gripper-5kg-on-c05kg',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/connector-single-suction-cup-on-10kg',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/connector-single-suction-cup-on-c05kg',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/connector-trigger-holder',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/crane-attachment-basket-1m',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/crane-attachment-basket-i50kg',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/crane-attachment-fork-lifter-2000kg',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/crane-attachment-vacuum-lifter-400kg',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/end-effector-bilian-bln-svk640-3r18-h20',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/end-effector-borunte-sponge-suction',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/end-effector-jodell',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/end-effector-jodell-els-150',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/end-effector-jodell-epg40-050',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/end-effector-jodell-evs01',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/end-effector-jodell-rg75-300',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/end-effector-robocon-drill',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/end-effector-shenzhen-ruichuang-130-590',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/fitting-g1-2f-to-12mm',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/fitting-g1-4f-to-12mm',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/fitting-g1-4f-to-8mm',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/fitting-g1-4m-to-12mm',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/fitting-r1-4m-to-8mm',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/gun-m134',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/gun-xm250-light',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/mewp-crane-basket-1m',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/mewp-crane-basket-i50kg',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/mewp-crane-fork-lifter-2000kg',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/mewp-crane-vacuum-lifter-400kg',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/motor-driver-bldc300-6012a-ns-48vdc',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/motor-gdz65-800-220',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/motor-jssby-114-4120-cd15-48vdc-500w',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/pump-24vdc-vacuum-pump',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/pump-cv-10hs',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/switch-dps-306rx-negative-pressure',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/switch-ps-1h-nv',
        },
        {
          type: 'doc',
          id: 'hardware/end-effector/switch-ps-1l-nv',
        },
      ],
    },
  ],
};

export default sidebars;

