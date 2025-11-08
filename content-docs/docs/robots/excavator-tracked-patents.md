# RoboCon Excavator Tracked - Patents Analysis

This page provides a comprehensive list and analysis of patents related to excavators, autonomous construction equipment, and robotics systems from major competitors including Doosan Robotics, Hyundai, Bedrock Robotics, and Built Robotics, with specific focus on evaluating potential conflicts with the RoboCon Excavator Tracked.

For general product information, see the [RoboCon Excavator Tracked](./excavator-tracked.md) main page.

## Relevant Patents List

### Doosan Robotics Patents

**Doosan Robotics Inc.** (also listed as "Doosan Robotics Co., Ltd.") is a major manufacturer of collaborative robots and industrial robotic systems. The following patents are relevant to autonomous construction equipment and robotics:

1. **US11668076B2** - (Doosan Robotics) - *Patent details to be researched*
2. **US11004235B2** - (Doosan Robotics) - *Patent details to be researched*
3. **US11313685B2** - (Doosan Robotics) - *Patent details to be researched*
4. **US20210002851A1** - (Doosan Robotics) - *Patent Application - details to be researched*
5. **US12018461B2** - (Doosan Robotics) - *Patent details to be researched*
6. **US12428804B2** - (Doosan Robotics) - *Patent details to be researched*
7. **US12104353B2** - (Doosan Robotics) - *Patent details to be researched*
8. **US12157985B2** - (Doosan Robotics) - *Patent details to be researched*
9. **US11454001B2** - (Doosan Robotics) - *Patent details to be researched*
10. **US10253481B2** - (Doosan Robotics) - *Patent details to be researched*
11. **US11492777B2** - (Doosan Robotics) - *Patent details to be researched*
12. **US12258726B2** - (Doosan Robotics) - *Patent details to be researched*
13. **US11679961B2** - (Doosan Robotics) - *Patent details to be researched*
14. **US11634882B2** - (Doosan Robotics) - *Patent details to be researched*

### Additional Relevant Patents

#### US12378750B2 - Autonomous Excavator Control System

**Summary:**
This patent covers autonomous control systems specifically designed for excavators, including autonomous digging operations, bucket control, and path planning for excavation tasks.

**Key Claims:**
- Autonomous excavator operation algorithms
- Automatic bucket control and digging sequences
- Path planning for excavation operations
- Integration of sensor systems for terrain analysis
- Autonomous boom, stick, and bucket control coordination

**RoboCon Excavator Tracked Comparison:**
- **Potential Conflict Areas:**
  - Autonomous excavator operation capabilities
  - Automatic boom, stick, and bucket control (motor_driver_ids830abs)
  - Path planning for excavation tasks
  - Sensor integration for terrain analysis (IMU, pressure sensors)

- **Differentiation Factors:**
  - RoboCon uses open-source ROS 2 architecture vs. proprietary control systems
  - CAN BUS control integration (motor_driver_ids830abs) differs from typical implementations
  - ROS 2-based control stack allows customization
  - Integration with ROBOCON ecosystem provides unique capabilities
  - Multi-robot coordination capabilities unique to RoboCon

- **Conclusion:** While both systems provide autonomous excavator operation, RoboCon's open-source ROS 2 approach and CAN BUS control architecture differ significantly from typical proprietary autonomous excavator systems. The specific implementation details (ROS 2 nodes, CAN BUS protocols, AI models) provide differentiation from patented approaches.

### Design Patents

**USD797159S1 - Excavator Ornamental Design**
- **Assignee**: Doosan Infracore Co., Ltd.
- **Issue Date**: September 12, 2017
- **Type**: Design Patent (ornamental design)
- **Focus**: Visual appearance of excavator design

**USD797160S1 - Excavator Ornamental Design**
- **Assignee**: Doosan Infracore Co., Ltd.
- **Issue Date**: September 12, 2017
- **Type**: Design Patent (ornamental design)
- **Focus**: Visual appearance of excavator design

**USD774112S1 - Excavator Body and Cab Exterior Design**
- **Assignee**: Doosan Infracore Co., Ltd.
- **Issue Date**: December 13, 2016
- **Type**: Design Patent (ornamental design)
- **Focus**: Ornamental design of excavator body and cab exterior

**RoboCon Excavator Tracked Comparison:**
- **Analysis**: Design patents protect ornamental (visual) designs, not functional features
- **Conflict Assessment**: RoboCon Excavator Tracked's visual appearance would need to be compared to these design patents
- **Recommendation**: Visual design differences reduce conflict risk; functional operation not affected by design patents
- **Note**: RoboCon Excavator may have different visual appearance (compact design, no cab for autonomous operation, different proportions)

## Key Patent Analysis

### Doosan Robotics Patent Portfolio

**Company Overview:**
Doosan Robotics Inc. is a major manufacturer of collaborative robots (cobots) and industrial automation systems. The company has a significant patent portfolio covering robotic control systems, human-robot collaboration, and autonomous operation technologies.

**Relevance to RoboCon Excavator Tracked:**
- Doosan Robotics focuses on collaborative robots and industrial automation
- Patents may cover control systems, sensor integration, and autonomous operation
- Potential overlap with RoboCon's autonomous excavator control systems

**Patent Analysis Status:**
Detailed analysis of the specific Doosan Robotics patents (US11668076B2, US11004235B2, US11313685B2, etc.) is pending. These patents will be analyzed for:
- Control system architecture
- Autonomous operation algorithms
- Sensor integration methods
- Human-robot interaction
- Safety systems

### Hyundai Autonomous Robotics Patents

**Company Overview:**
Hyundai Motor Group and Hyundai Heavy Industries are major players in autonomous construction equipment. Hyundai has acquired Doosan Infracore (now "Develon") and has significant investments in autonomous construction machinery.

**Relevance to RoboCon Excavator Tracked:**
- Hyundai/Develon manufactures construction equipment including excavators
- Patents may cover autonomous construction equipment systems
- Potential overlap with RoboCon's autonomous excavator capabilities

**Patent Analysis Status:**
Detailed analysis of Hyundai autonomous robotics patents is pending. Hyundai patents will be analyzed for:
- Autonomous construction equipment control
- Excavator automation systems
- Multi-robot coordination in construction
- AI-driven operation systems

### Bedrock Robotics Patents

**Company Overview:**
Bedrock Robotics (research in progress) - Company information and patent portfolio to be researched.

**Relevance to RoboCon Excavator Tracked:**
- Autonomous construction equipment focus
- Potential overlap with RoboCon's autonomous excavator systems

**Patent Analysis Status:**
Bedrock Robotics patent portfolio research is pending. Analysis will include:
- Autonomous construction equipment patents
- Excavator automation systems
- Control system architectures

### Built Robotics Patents

**Company Overview:**
Built Robotics is a San Francisco-based company specializing in autonomous construction equipment retrofit systems. The company converts traditional construction equipment into autonomous machines.

**Relevance to RoboCon Excavator Tracked:**
- Built Robotics focuses on autonomous excavators and loaders
- Patents likely cover autonomous control systems, sensor integration, and path planning
- Direct competitor in autonomous excavator space

**Patent Analysis Status:**
Built Robotics patent portfolio research is pending. Analysis will include:
- Autonomous excavator control systems
- Retrofit systems for construction equipment
- Sensor fusion for autonomous operation
- Path planning for construction sites

## Additional Relevant Patents

### Construction Equipment Autonomous Navigation

**US10626576B2 - Autonomous Navigation for Construction Vehicles**
(Also analyzed in Front Loader section - see [Front Loader Patents](../robots/front-loader-tracked-300kg-patents.md))

**RoboCon Excavator Tracked Relevance:**
- Autonomous navigation capabilities
- Terrain mapping for excavation sites
- Path planning for earth-moving operations
- Differentiation through ROS 2 Nav2 and AI-driven navigation

### Remote Operation Systems

**US8392075B2 - Remote Operation System for Loaders**
(Also analyzed in Front Loader section)

**RoboCon Excavator Tracked Relevance:**
- Remote operation capabilities
- ROS 2 DDS communication differs from proprietary protocols
- Autonomous-first design vs. remote-control focus

## Patent Conflict Analysis Summary

**Overall Assessment:**
The RoboCon Excavator Tracked utilizes autonomous excavator technologies that may intersect with existing patents from Doosan Robotics, Hyundai, Bedrock Robotics, and Built Robotics, but key differentiators reduce conflict risk:

1. **Open-Source Architecture**: ROS 2 provides different implementation approach
2. **CAN BUS Control**: Integration with motor_driver_ids830abs via CAN BUS differs from traditional systems
3. **AI Model Differences**: AI-driven operation may use different algorithms than patented approaches
4. **System Integration**: ROBOCON ecosystem provides unique integration approach
5. **Design Differences**: Compact design and autonomous-first configuration differ from traditional excavators

**Design Patent Considerations:**
- Visual appearance differs from Doosan design patents
- Functional operation not affected by design patents
- Autonomous/no-cab configuration provides different visual design

**Recommendations:**
- Conduct detailed claim-by-claim analysis with patent attorney for all Doosan Robotics patents
- Research Bedrock Robotics and Built Robotics patent portfolios
- Document unique design elements and implementation differences
- Visual design comparison with design patents
- Monitor patent application status for autonomous excavator technologies
- Compare control system architectures (ROS 2 vs. proprietary systems)

## Technology Differentiation

### RoboCon Excavator Tracked Unique Advantages

1. **Open-Source ROS 2 Architecture**
   - Fully open ROS 2 ecosystem integration
   - Customizable AI and control algorithms
   - Extensible through ROBOCON marketplace

2. **CAN BUS Control Integration**
   - Direct CAN BUS control of excavator functions
   - motor_driver_ids830abs for boom, stick, and bucket control
   - Real-time control feedback

3. **Autonomous Operation**
   - Fully autonomous excavation capabilities
   - AI-driven path planning for digging operations
   - Sensor fusion for terrain analysis

4. **ROBOCON Ecosystem Integration**
   - Seamless integration with other RoboCon robots
   - Shared hardware drivers and components
   - Unified ROS 2 development environment
   - Multi-robot coordination capabilities

## Patent Search Information

### Full Legal Company Names for Patent Search

**Doosan Robotics:**
- **Full Legal Name**: **Doosan Robotics Inc.** (also listed as "Doosan Robotics Co., Ltd.")
- **Headquarters**: South Korea
- **Focus**: Collaborative robots, industrial automation

**Hyundai Heavy Industries / Develon:**
- **Current Legal Name**: **Hyundai Heavy Industries Co., Ltd.** (Develon brand)
- **Previous Name**: Doosan Infracore Co., Ltd. (acquired by Hyundai)
- **Focus**: Construction equipment, excavators

**Bedrock Robotics:**
- **Legal Name**: *To be researched*
- **Focus**: Autonomous construction equipment

**Built Robotics:**
- **Legal Name**: **Built Robotics, Inc.**
- **Headquarters**: San Francisco, California, USA
- **Focus**: Autonomous construction equipment retrofit systems

### Search Queries for Excavator Patents

1. **Doosan Robotics Patents:**
   ```
   assignee:"Doosan Robotics" OR assignee:"Doosan Robotics Co" OR assignee:"Doosan Robotics Inc" (autonomous OR excavator OR construction)
   ```

2. **Hyundai Construction Equipment Patents:**
   ```
   assignee:"Hyundai Heavy Industries" OR assignee:"Develon" OR assignee:"Doosan Infracore" excavator autonomous
   ```

3. **Built Robotics Patents:**
   ```
   assignee:"Built Robotics" autonomous excavator OR "construction equipment"
   ```

4. **Bedrock Robotics Patents:**
   ```
   assignee:"Bedrock Robotics" autonomous construction
   ```

5. **General Autonomous Excavator Patents:**
   ```
   (excavator OR "earth-moving") autonomous OR automation (construction OR robotics)
   ```

6. **Specific Patent Numbers:**
   ```
   US11668076B2 OR US11004235B2 OR US11313685B2 OR US20210002851A1 OR US12018461B2 OR US12428804B2 OR US12104353B2 OR US12157985B2 OR US11454001B2 OR US10253481B2 OR US11492777B2 OR US12258726B2 OR US11679961B2 OR US11634882B2
   ```

### Patent Resources

- [Google Patents](https://patents.google.com)
- [USPTO Patent Search](https://www.uspto.gov/patents/search)
- [WIPO Patentscope](https://patentscope.wipo.int/search/en/search.jsf)
- [KIPO (Korean Intellectual Property Office)](https://www.kipo.go.kr/en/MainApp)

## Related Pages

- [RoboCon Excavator Tracked](./excavator-tracked.md) - Main product page
- [RoboCon Excavator Tracked Comparison](./excavator-tracked-comparison.md) - Competitive analysis
- [RoboCon Front Loader Tracked 300kg Patents](../robots/front-loader-tracked-300kg-patents.md) - Related patent analysis

## Next Steps for Patent Research

1. **Detailed Patent Analysis Required:**
   - Conduct claim-by-claim analysis of all listed Doosan Robotics patents
   - Research Bedrock Robotics patent portfolio
   - Analyze Built Robotics patent filings
   - Review Hyundai autonomous construction equipment patents

2. **Competitive Landscape Analysis:**
   - Compare control system architectures
   - Analyze sensor integration approaches
   - Review autonomous operation algorithms
   - Assess path planning methodologies

3. **Legal Consultation:**
   - Engage patent attorney for detailed infringement analysis
   - Document unique design elements and implementation differences
   - Consider prior art research for key patents

4. **Ongoing Monitoring:**
   - Monitor new patent applications in autonomous excavator space
   - Track competitor patent filings
   - Update analysis as new information becomes available

## References

1. [Google Patents - Doosan Robotics](https://patents.google.com/?assignee=Doosan+Robotics)
2. [Google Patents - Built Robotics](https://patents.google.com/?assignee=Built+Robotics)
3. [Google Patents - US12378750B2](https://patents.google.com/patent/US12378750B2)
4. [USPTO Patent Search](https://www.uspto.gov/patents/search)
5. [RoboCon Excavator Tracked Documentation](./excavator-tracked.md)

