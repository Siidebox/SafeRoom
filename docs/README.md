# Documentation

## Understanding the system

| Document | Contents |
|---|---|
| [architecture.md](architecture.md) | System layers, the MSS/DSS firmware split, the signal pipeline, the TLV protocol, and the UART bandwidth budget |
| [hardware.md](hardware.md) | Bill of materials, mounting geometry, flashing, and the runtime `.cfg` reference |
| [dashboard.md](dashboard.md) | How the monitoring dashboard is built and how to publish events to it |
| [limitations.md](limitations.md) | What this approach cannot do, and why |
| [ml_model_rationale.md](ml_model_rationale.md) | Why a learned model exists alongside the rule-based detector |

## Protocols

| Document | Contents |
|---|---|
| [data_collection_protocol.md](data_collection_protocol.md) | Class targets, required variation, session procedure, capture-day schedule |
| [labeling_protocol.md](labeling_protocol.md) | Label vocabulary, boundary criteria, quality requirements |

## Operating it

The [runbook](runbook/README.md) holds the day-to-day commands: dashboard, live
view, recording, labelling, training, the end-to-end deployment test,
troubleshooting, and Telegram alerts.

## Firmware

[`code/People_Tracking/3D_People_Tracking/README.md`](../code/People_Tracking/3D_People_Tracking/README.md)
covers the upstream TI source, the SafeRoom modifications, and how to build and
flash the image.
