# Planning — 수도권 Gray-zone C-UAS 의사결정 체계 (SAPIENT + Stone Soup)

> 과제 03: 도심 환경에서 소형 드론·풍선 등 저신호(LSS: Low-altitude, Slow, Small) 비행체를
> **탐지 → 추적 → 식별 → 위협평가 → 대응 결심 지원**까지 수행하는 Gray-zone Counter-UAS 체계

---

## 1. 범위와 핵심 아이디어

**범위:** 센서(ASM: Autonomous Sensor Module)는 SAPIENT로 탐지 정보를 *주어지는 입력*으로 간주한다.
이 프로젝트는 그 이후 단계, 즉 **Fusion / Decision-Making Node(DMM)의 의사결정 체계 구축**에 집중한다.

| 구성 요소 | 역할 |
|---|---|
| **SAPIENT (BSI Flex 335 v2.0)** | ASM ↔ DMM ↔ 이펙터 간 표준 인터페이스. 입력(`DetectionReport`)과 출력(`Alert`, `Task`)의 규격 |
| **Apex Middleware** | 메시지 라우팅·스키마 검증·SQLite 기록·Replay |
| **Stone Soup** | DMM 내부의 다중센서·다중표적 추적 엔진 → 단일 공중상황도(트랙) 생성 |
| **위협평가·결심지원 엔진** | 트랙 → 식별 → 위협 점수 → **저위험 대응 옵션 추천** (사람이 최종 결심) |

**한 줄 요약:** 여러 ASM이 SAPIENT로 보내는 단편적 탐지를 Stone Soup이 하나의 상황도로 융합하고,
의사결정 엔진이 골든타임 안에 "무엇이, 얼마나 위험하며, 어떻게 대응할지"를 근거와 함께 지휘관에게 추천한다.

---

## 2. 시스템 아키텍처

```
  ASM들 (주어진 입력 / 개발 단계에서는 SapientAsmSimulator·자체 시나리오 생성기)
      │  SAPIENT v2 (protobuf over TCP, 4-byte little-endian length prefix)
      │  Registration / StatusReport / DetectionReport        ↑ Task / TaskAck
      ▼
 ┌─────────────────────┐
 │  Apex Middleware     │  라우팅 · 검증 · 기록 · Replay  (Child port 5020: v2.0)
 └──────────┬──────────┘
            ▼
 ┌────────────────────── Decision-Making Node (본 프로젝트) ──────────────────────┐
 │ ① SAPIENT Adapter    : DetectionReport → Stone Soup Detection (+센서 메타데이터)  │
 │ ② Track Fusion       : Stone Soup 다중센서·다중표적 추적                          │
 │ ③ Identification     : 센서별 분류 + 운동 특성 → 표적 유형 확률 (드론/풍선/조류…)  │
 │ ④ Threat Assessment  : 보호구역·도달시간·행동·유형 → 위협 점수                    │
 │ ⑤ Response Planner   : 대응 옵션 사다리 + 부수피해 추정 → 추천안                   │
 │ ⑥ Sensor Tasking     : 불확실 트랙 해소용 Task (look_at / follow / threshold)     │
 └───────────┬───────────────────────────────────────────────┬────────────────┘
             ▼ Alert (추천안·근거)                            ▼ Task (승인 후)
       C2 GUI (상황도 · 경보 · 승인/거부)  ── 승인 ──▶  ASM / 이펙터 노드
```

### 2.1 SAPIENT 메시지 매핑 (v2.0, `SAPIENT_MODULES/SAPIENT-Proto-Files/bsi_flex_335_v2_0`)

| 방향 | 메시지 / 필드 | DMM에서의 사용 |
|---|---|---|
| 입력 | `Registration.node_definition`, `capabilities` | 센서 종류(`NODE_TYPE_CAMERA`/`RADAR`/`PASSIVE_RF`/`ACOUSTIC`…)별 측정모델·신뢰도 설정 |
| 입력 | `StatusReport` | 센서 가용성·위치 → 커버리지 공백 파악 |
| 입력 | `DetectionReport.range_bearing` / `location`, `detection_confidence` | 추적기 측정값 |
| 입력 | `DetectionReport.classification[]`, `signal[]`, `behaviour[]` | 식별·위협평가 특징 |
| 출력 | `DetectionReport` (DMM 명의, `object_id`=트랙 ID, `enu_velocity`, `prediction_location`) | 융합 트랙을 상위 C2에 보고 |
| 출력 | `Alert {alert_type, priority, ranking, confidence, region_id, associated_detection, additional_information}` | 위협 경보 + 추천 대응안·근거 |
| 출력 | `Task.command.look_at` / `follow` / `detection_threshold`, `Task.region` | 센서 큐잉, 보호구역(`AREA_OF_INTEREST`)·무시구역(`IGNORE`) 배포 |

### 2.2 추적 — Stone Soup 구성 (초안)

| 모듈 | 후보 구현 | 비고 |
|---|---|---|
| 운동 모델 | `CombinedLinearGaussianTransitionModel` (3D CV) | 드론: 큰 프로세스 노이즈 / 풍선: 풍속 기반 저노이즈 |
| 측정 모델 | 레이더: `CartesianToElevationBearingRange`, 방위 전용 센서: bearing-only 모델 | `Registration` 노드 타입으로 자동 선택 |
| 필터 | EKF/UKF, bearing-only 다중센서 시 Particle Filter 검토 | |
| 연관 | GNN(`GNNWith2DAssignment`) → 밀집 시 JPDA | 조류·클러터 대응 |
| 개시/삭제 | `MultiMeasurementInitiator`, `CovarianceBasedDeleter` | 오경보 억제 vs 골든타임 트레이드오프 |
| 센서 관리 | `stonesoup.sensormanager` | 공분산 큰 트랙에 `look_at` Task |
| 평가 | `stonesoup.metricgenerator` (OSPA, SIAP) | |

### 2.3 식별

- ASM 분류 확률(`classification[].confidence`)을 센서 신뢰도로 가중해 **베이지안 결합**, 트랙 단위로 누적.
- 운동 기반 특징 추가: 풍선은 바람과 같은 속도로 **수동 표류**, RF 신호 없음 → 트랙 속도 vs 기상 풍속 정합도.
- 출력: 유형별 확률 + 식별 신뢰도 (낮으면 ⑥ Sensor Tasking으로 추가 관측 요청).

### 2.4 위협평가

- **위협 점수** (0–1 → `Alert.ranking`) = f(유형 확률, 보호구역까지 거리·예상 도달시간, 접근 여부, 고도·속도, 배회/체공 행동, 탑재물 추정)
- **보호구역**: 대통령실·주요 국가시설·인구 밀집지 → 구역별 가중치·허용 고도 설정
- 해석 가능성 우선: 처음에는 **규칙 기반 + 가중합**으로 구현하고, 점수마다 기여 요인을 근거로 함께 출력.

### 2.5 대응 결심 지원

- **대응 옵션 사다리** (저위험 → 고위험). 옵션마다 *효과 / 부수피해 / 소요시간 / 법적 제약*을 평가:
  1. 감시 지속 (`follow`)
  2. 경고·유관기관 통보
  3. 비물리적 무력화 (RF 재밍, GNSS 교란 — 주변 민간 통신 영향 평가)
  4. 포획 (그물·요격 드론 / 풍선: 안전지역 하강 후 회수)
  5. 물리적 타격 (도심 최후 수단)
- **추천 논리**: 위협 점수와 남은 골든타임으로 필요한 최소 대응 수준을 정하고, 그 수준 이상의 옵션 중 부수피해가 가장 작은 것을 추천.
- **Human-in-the-loop**: 추천안·근거·대안을 `Alert`로 제시하고, 실행 `Task`는 지휘관 승인 후에만 발행. 결심 이력은 전부 기록.

---

## 3. 저장소 구조 (계획)

```
D4D_2/
├── planning.md
├── SAPIENT_MODULES/                 # dstl 원본 (수정 금지, 참조용)
│   ├── SAPIENT-Proto-Files/         # v2.0 proto
│   ├── Apex-SAPIENT-Middleware/     # 미들웨어
│   └── BSI-Flex-335-v2-Test-Harness/# v2 테스트 하네스 (SapientAsmSimulator 포함)
├── sapient_msg/                     # v2.0 proto → Python 컴파일 결과
├── dmm/                             # Decision-Making Node
│   ├── adapter/                     # SAPIENT ↔ Stone Soup 변환
│   ├── tracking/                    # Stone Soup 추적기 구성
│   ├── identification/
│   ├── threat/
│   ├── response/
│   └── tasking/
├── scenarios/                       # 도심 시나리오 → SAPIENT DetectionReport 재생기
├── c2_gui/                          # 상황도 · 경보 · 승인 UI
└── tests/
```

---

## 4. 단계별 실행 계획

| # | 단계 | 산출물 | 검증 기준 |
|---|---|---|---|
| 1 | **SAPIENT 통신 기반** | v2.0 proto 컴파일, DMM 클라이언트(Apex 연결·Registration) | Apex에 DMM 등록, `SapientAsmSimulator` 탐지가 DMM까지 수신 |
| 2 | **시나리오 입력** | 도심 시나리오(드론 접근·배회, 풍선 표류, 조류 클러터) → 다중 ASM `DetectionReport` 생성기, ground truth 동시 출력 | Apex DB에 기록되고 Replay 가능 |
| 3 | **추적 융합** | SAPIENT Adapter + Stone Soup 추적기 | OSPA 수렴, 트랙 단절·오트랙 수 측정 |
| 4 | **식별** | 분류 융합 + 운동 특징 | 드론/풍선/조류 혼동행렬, 목표 정확도(예: ≥90%) |
| 5 | **위협평가** | 위협 점수 + 근거 출력, 보호구역 설정 | 시나리오별 기대 위협 순위와 일치 |
| 6 | **대응 결심 지원** | 옵션 사다리·부수피해 평가·추천, `Alert` 발행 | 시나리오별 기대 대응안과 일치, 탐지→추천 지연시간 |
| 7 | **센서 태스킹 (폐루프)** | 불확실 트랙에 `look_at`/`follow` Task | TaskAck 수신, 식별 신뢰도 상승 확인 |
| 8 | **C2 GUI + 데모** | 상황도·경보·승인/거부 화면 | 탐지→추적→식별→결심 End-to-End 시연 |
| 9 | **적합성 검증** | v2 Test Harness로 DMM 메시지 검증 | 하네스 검증 통과 |

**핵심 성능 지표(KPI):** 탐지→추천 지연(골든타임), 식별 정확도, 오경보율, 트랙 연속성(OSPA), 추천 대응의 적절성·부수피해 수준.

---

## 5. 가정 및 확인 필요 사항

- **SAPIENT 버전**: BSI Flex 335 **v2.0**.
- **입력**: ASM 출력(`DetectionReport`)은 주어진다고 가정. 개발·평가는 `SapientAsmSimulator`와 자체 시나리오 생성기로 대체.
- **ASM 구성**: 실제 운용을 가정할 ASM 종류·배치(레이더/EO/RF/음향 조합)를 시나리오 설계 전에 확정 필요.
- **ASM 출력 수준**: SAPIENT는 센서별 프로파일이 없고 ASM이 Registration(`tracking_type` 등)으로 선언 → DMM은 측정 융합·트랙 융합을 모두 지원. 상세: [docs/sapient_sensor_assumptions.md](docs/sapient_sensor_assumptions.md)
- **대응 수단**: 재밍·GNSS 교란 등은 법적 제약 → 추천·시뮬레이션까지만 수행 가정.
- **위협평가 기준**: 보호구역 목록·가중치·대응 교리(ROE)는 공개 정보 기반 가정값으로 시작.
- **언어**: Apex·Stone Soup 모두 Python → DMM도 Python. (Test Harness는 C#/.NET — 실행 환경 필요)
