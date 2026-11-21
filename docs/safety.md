# Safety

## Design principles

1. **AI does not own the actuator.**  
   The Linux AI engine returns a recommendation. The STM32 applies policy and drives relays.

2. **Deterministic limits override AI.**  
   Hard electrical rules (current, voltage, PF bounds, etc.) can force a safe action regardless of model output.

3. **Independent emergency stop.**  
   A dedicated E-STOP input (prototype: NC on D5) forces all relay channels OFF in firmware.

4. **Priority-aware shedding.**  
   Lower-priority loads are candidates for disconnection before critical loads.

## Prototype limitations

- Developed and demonstrated at **low ampere** laboratory levels.
- Not a substitute for certified protection relays, fuses, or contactors sized for the installation.
- RS-485 path in the prototype is TX-only status reporting.
- Model performance on arbitrary industrial sites is **not** statistically validated in the current release.

## Recommended practice for experimenters

- Always use a separate upstream protective device.
- Commission with dummy loads or limited current sources first.
- Verify E-STOP and "all relays OFF" behaviour before energising meaningful loads.
- Treat every AI decision as advisory until your own validation is complete.
