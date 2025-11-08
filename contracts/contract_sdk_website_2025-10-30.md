
# Contract: SDK Documentation Site Configuration
**Date:** 2025-10-30  

---

## Parties
- **Contractor:** Ammar Iqbal (@ammar1818)  
- **Client:** ROBOCON INC  

---

## Project Overview
The Contractor agrees to configure and deploy a **Docusaurus-based SDK documentation website** for the Client, **ROBOCON INC**, hosted at:

**➡️ https://sdk.roboconinc.com**

This SDK documentation site will serve as the primary technical reference for developers programming applications for **ROBOCON construction machinery**. The documentation will mirror the structure and depth of the **Unitree G1 SDK Development Guide**, located at:  
[https://support.unitree.com/home/en/G1_developer/basic_motion_routine](https://support.unitree.com/home/en/G1_developer/basic_motion_routine)

---

## Scope of Work
1. **Framework Setup**
   - Install and configure **Docusaurus** (React-based documentation framework).
   - Configure navigation, search, and theme consistent with ROBOCON branding.
   - Implement version control for SDK versions (e.g., v1.0, v2.0).
   - Configure **Nginx web server** on AWS EC2 for production hosting.

2. **SSL/TLS Configuration**
   - Deploy **OpenSSL-based certificate setup** for HTTPS across all Robocon domains.
   - SSL certificates will be **purchased and installed** similarly to the existing `roboconinc.com` configuration, using `.key`, `.pem`, `.crt`, `.ca-bundle`, and `.p7b` files.
   - Certificates will be issued for the following domains:
     - `sdk.roboconinc.com`
     - `support.roboconinc.com`
     - `roboconinc.com`
     - `www.roboconinc.com`
   - Ensure automatic renewal or replacement process via AWS or manual OpenSSL renewal workflow.

3. **Content Integration**
   - Create initial documentation structure including:
     - SDK Overview
     - Getting Started Guide
     - API Reference (Motor Control, AI Integration)
     - ROS 2 & Nav 2 Architecture
     - Installation & Deployment Guide
     - Marketplace Integration (AI Programs section)
   - Import example documentation inspired by Unitree SDK for low-level motor control routines.

4. **ROBOCON Architecture Integration**
   - Outline SDK integration with **ROBOCON OS**, including:
     - ROS 2 Nodes and Topics used
     - Nav 2 Path Planning modules
     - AI Program packaging, deployment, and runtime on Robocon OS
   - Support both **ROBOCON-manufactured** and **third-party robots** capable of running Robocon OS.

5. **Marketplace Integration Section**
   - Add documentation for AI program packages available on:  
     [https://roboconinc.com/marketplace](https://roboconinc.com/marketplace)
   - Include process for packaging and submitting AI programs.

6. **Deployment**
   - Host the final documentation site at **sdk.roboconinc.com** using AWS EC2.
   - Ensure responsive, mobile-friendly design with search functionality.
   - Implement SSL/TLS through **OpenSSL** configuration on EC2.

---

## Deliverables
- Fully functional Docusaurus site configured and deployed at sdk.roboconinc.com  
- GitHub repository with source Markdown files and Docusaurus config  
- HTTPS-enabled domain secured via OpenSSL certificates for all Robocon domains  
- Documentation covering SDK, ROS 2, and AI Program architecture  

---

## Payment
- **Total Amount:** $200 (USD)  
- **Payment Due:** Upon successful deployment of sdk.roboconinc.com with SSL enabled

---

## Ownership and Rights
- All source code, documentation, and configurations created under this contract become the exclusive property of **ROBOCON INC** upon payment.
- The Contractor may list the project in their portfolio with ROBOCON’s written consent.

---

## Signatures

**Contractor:**  
Name: Ammar Iqbal  
Handle: @ammar1818  
Date: ____________________  

**Client:**  
Name: Authorized Representative, ROBOCON INC  
Date: ____________________  

---
