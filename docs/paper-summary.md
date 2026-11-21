# Paper summary (AIIE 2026)

**Title:** NeuralGrid: A Dual-Processor Edge AI Framework for Predictive Electrical Load Management and Priority-Aware Control  

**Authors:** Syed Umar Ali, Muhammad Sameer Ahmed, Syeda Noor Fatima  
**Affiliation:** IIEE, NED University of Engineering & Technology, Karachi, Pakistan  
**Abstract ID:** 114171982  

## Gap

Monitoring reports the present state. Threshold protection trips at fixed limits. Neither alone necessarily interprets evolving behaviour or ranks loads for selective intervention.

## Contribution

An embedded integration architecture that links:

- temporal electrical feature analysis,
- complementary ML evidence (classifier + anomaly),
- deterministic safety constraints,
- priority-aware load shedding,

on a dual-processor edge platform **without** granting the learned model unrestricted actuator authority.

## Research question

How effectively can temporal electrical assessment and safety-constrained decision fusion improve selective load management compared with threshold-only logic?

## Evidence boundary

The prototype demonstrates functional end-to-end integration and scenario behaviour. Formal quantitative benchmarks (accuracy, false-alarm rate, latency) are planned future work.
