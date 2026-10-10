# SAPIENT 센서(ASM)별 전송 수준 조사

> 목적: DMM 설계 시 "각 ASM이 어느 수준까지 처리해서 보내는가"에 대한 가정을 정리한다.
> 표기: **[출처]** = 문서·코드로 확인, **[추론]** = 출처 기반 추론.
> 한계: BSI Flex 335 v2.0 본문은 유료라 직접 확인하지 못함. 규범 내용은 무료 공개된 전신인 **SAPIENT ICD v7.0**과
> dstl 공식 **v2.0 `.proto`**, **v2 Test Harness 검증 코드**로 확인함.

---

## 1. 결론 요약

1. **SAPIENT는 센서 종류별 전송 프로파일을 정의하지 않는다.** **[출처]**
   - 메시지는 일부러 일반화되어 있음 — "no one sensor will be expected to fill in all of the fields" (ICD v7 §4.1).
   - 각 ASM이 **Registration에서 자신이 채울 필드를 선언**하고, 해당 필드만 보낸다 (ICD v7 §5.5).
2. **ASM은 "정보"를 보내지 원시 데이터를 보내지 않는다.** **[출처]**
   - 탐지·분류는 엣지에서 AI로 수행, "only the information not the raw data" (gov.uk/sapient).
   - 영상·포인트클라우드는 필요 시 URL(`associated_file`)로만 참조 (ICD v7 §2.1.4).
3. **추적(Track) 여부는 ASM마다 다르며 Registration의 `tracking_type`으로 선언한다.** **[출처]**
   - 생략 시 NONE으로 간주 (ICD v7 §5.4.3).
4. **따라서 DMM은 센서 종류로 하드코딩하지 말고, Registration을 읽어 처리 방식을 동적으로 결정해야 한다.** **[추론]**

---

## 2. 모든 ASM 공통 — 최소 보장 수준 **[출처: v2.0 proto + Test Harness 검증기]**

| 항목 | 내용 |
|---|---|
| **필수** | `report_id`(ULID), `object_id`(ULID), 위치 **`range_bearing` 또는 `location` 중 정확히 하나** |
| 선택 | `detection_confidence`, `classification[]`, `behaviour[]`, `enu_velocity`, `track_info`, `prediction_location`, `object_info`, `signal[]`, `associated_file`, `associated_detection`, `derived_detection`, `colour`, `id` |
| 속도 | v2.0에서는 **ENU 속도만** 사용 (다른 좌표계 속도는 폐지) |
| 좌표 | ASM의 *native 형식* 사용. 구면(거리·방위·고각, 센서 기준) 또는 직교(위경도/UTM + datum). Registration에서 선언한 형식을 일관 사용 (ICD v7 §4.2) |
| 픽셀 좌표 | 카메라 픽셀 좌표는 **실세계 단위 + 오차로 변환 후 보고**해야 함 (ICD v7 §4.2) |
| 센서 위치 | 반드시 직교좌표 (`StatusReport`) |
| Heartbeat | `status_interval` 보통 1–60초. 누락 시 DMM은 노드 상실로 판단 가능 (ICD v7 §5.3.1) |
| 지연 | `maximum_latency` (모드별, 선택). **수치 요구사항 없음** |

> 즉, 최악의 경우 DMM이 받을 수 있는 정보는 **"ID + 위치(또는 방위만) "** 뿐이다.

### Registration에서 DMM이 읽어야 할 선언 항목

| 필드 | DMM에서의 의미 |
|---|---|
| `node_definition.node_type` / `node_sub_type` | 센서 종류 (RADAR, CAMERA, PASSIVE_RF, ACOUSTIC, LIDAR…) — sub_type은 자유 문자열 |
| `mode_definition.tracking_type` | NONE / TRACKLET / TRACK / TRACK_WITH_RE_ID → **측정 융합 vs 트랙 융합** 결정 |
| `detection_definition.location_type` | range-bearing인지 location인지 → **측정모델** 결정 |
| `detection_definition.geometric_error` | 위치 오차 특성 (예: 거리에 선형 비례) → **측정 노이즈 공분산** |
| `detection_definition.detection_performance` | 오경보율(FAR) 등 → 클러터 밀도·트랙 개시 기준 |
| `detection_class_definition` | 보고 가능한 분류 목록, `SINGLE_CLASS`(최상위 1개만) vs `MULTI_CLASS`(전 클래스 확률) |
| `velocity_type` | 속도 제공 여부 |
| `scan_type` | FIXED / SCANNING / STEERABLE → 갱신 주기, `look_at` 가능 여부 |
| `mode_definition.task` | 받을 수 있는 Task 명령 (look_at, follow, threshold…) |
| `maximum_latency` | 측정 시각 보정, 골든타임 예산 |

---

## 3. 센서 종류별 예상 전송 수준

표준이 센서별 내용을 정하지 않으므로, 아래 "예상 내용"은 대부분 **[추론]**이다. 출처는 해당 열에 따로 표시.

| 센서 | 근거 자료 | 예상 위치 형식 | 예상 추적 수준 | 예상 분류·부가정보 |
|---|---|---|---|---|
| **레이더** | **[출처]** 2015 dstl 시험의 레이더 ASM: "광역 탐지·분류". 회전 레이더는 ICD의 SCAN_TYPE_SCANNING 예시 | 거리·방위·고각 또는 위경도 | TRACK / TRACKLET | 속도(ENU) 제공 가능성 높음. 분류는 거친 수준(UAV vs 조류 등) |
| **EO/IR 카메라** | **[출처]** 2015 시험 열상 ASM: "존재, 위치, 궤적, 행동" 보고. PTZ는 STEERABLE 예시. 스냅샷은 URL로 전달 | **방위·고각만** (거리측정·지오레퍼런싱이 없을 때) | TRACKLET | 분류가 가장 풍부함. `associated_file` 이미지 |
| **Passive RF** | **[출처]** `signal[]` 구조(진폭·중심주파수 필수), 분류체계의 "RF Transmitter → UAV Transmitter / UAS Control Station". R&S ARDRONIS가 TIE23에서 SAPIENT로 연동 | 방위(방향탐지) 또는 위치. 조종기 위치를 보고할 수도 있음 | 불확실 | 주파수·진폭, 프로토콜·기체 `id` 가능 |
| **음향** | **[출처]** NodeType 열거값만 있음 | 방위만 | NONE / TRACKLET | 신뢰도 낮음, 거친 분류 |
| **라이다** | **[출처]** 2015 시험 스캐닝 레이저 ASM: "형상, 추적, 설명", 구역 침범 플래그 | 3D 위치 | TRACK | `object_info`에 크기·형상 |

> NATO TIE(21–24) 관련 공개 자료에서도 업체들이 **필드 단위로 무엇을 보냈는지는 확인되지 않음**.

---

## 4. 분류체계(Taxonomy) — 본 과제에 중요한 사항

- 분류체계는 3단계 계층이며 **비규범(not normative)**이다. ASM은 가능한 가장 하위 단계를 보고해야 한다. **[출처: ICD v7 Appendix A]**
- `Air Vehicle`의 하위 분류는 Manned/UAV × Rotary/Fixed Wing, 각각 Commercial/Military로 나뉜다. `Animal → Bird`도 있다. **[출처: ICD v7, Test Harness `CommonValidator.cs`]**
- ⚠️ **"Balloon(풍선)" 클래스가 없다.** **[출처]**
  - v2.0의 `TaxonomyDockDefinition`으로 core 클래스에 확장 하위클래스를 붙이는 것이 의도된 방법으로 보인다. **[추론]**
  - 예: `sapient_core: Air Vehicle` 아래 자체 namespace의 `Balloon`.
- `"Unknown"`은 "모르겠다"가 아니라 **"알려지지 않은 클래스임이 확실하다"**는 뜻이다. **[출처]**
- 업체마다 자체 클래스 목록을 정의할 수 있다 (ICD v7 §4.5.2). → DMM은 **벤더별 클래스 문자열 매핑 테이블**이 필요하다. **[출처 + 추론]**
- ⚠️ **행동(behaviour) 목록이 지상 인원 중심이다.**
  - 허용값: walking, running, crawling, climbing, digging, throwing, loitering, active, passive, other.
  - v2 Test Harness 검증기는 이 목록 밖의 값을 **거부**한다 (`BehaviourValidator.cs`). **[출처]**
  - 드론의 hovering, 풍선의 drifting 등은 표준 값이 없으므로 `loitering`/`other`를 쓰거나 DMM 내부 속성으로 관리해야 한다. **[추론]**

---

## 5. DMM 설계에 대한 시사점 **[추론]**

1. **Registration 기반 센서 프로파일링**: 노드 등록 시 `tracking_type`, `location_type`, `geometric_error`, 클래스 정의를 파싱해 노드별 측정모델·노이즈·융합 방식을 자동 구성한다.
2. **두 가지 입력 모두 처리**
   - `tracking_type = NONE`: 탐지(측정)로 보고 Stone Soup에서 직접 추적한다.
   - `TRACKLET` / `TRACK`: ASM 트랙으로 보고 트랙-트랙 융합한다. 같은 `object_id`의 연속 보고는 해당 ASM 트랙의 갱신으로 처리한다.
3. **방위 전용(bearing-only) 센서**(카메라·음향·RF DF)는 단독으로는 거리를 모르므로, 다중 센서 삼각측량이나 레이더 트랙과의 연관이 필수다.
4. **분류 정보의 불균일성**
   - SINGLE_CLASS(최상위 1개 확률)와 MULTI_CLASS(전 클래스 확률)가 섞여 들어온다.
   - 클래스 문자열도 벤더마다 다르다.
   - → 내부 공통 클래스(드론-회전익 / 드론-고정익 / 풍선 / 조류 / 유인기 / 미상)로 정규화한 뒤 베이지안 결합한다.
5. **풍선 식별은 DMM 책임**: 표준 클래스가 없어 ASM이 풍선을 보고할 가능성이 낮다. 운동 특성(풍속 정합 표류)과 RF 부재 등으로 DMM이 추론해야 한다.
6. **시간 처리**: `maximum_latency`와 메시지 timestamp로 측정 시각을 보정한다. 지연 수치 요구가 없으므로 DMM이 자체 골든타임 예산을 정의한다.
7. **노드 상태 감시**: heartbeat 누락을 감지해 커버리지 공백을 상황도에 반영한다.

---

## 6. 시나리오·시뮬레이터 설계용 기본 가정 (제안)

본 프로젝트의 가상 ASM은 아래 수준으로 보낸다고 가정한다. 확정 전 팀 검토가 필요하다.

| 가상 ASM | `location_type` | `tracking_type` | 분류 | 기타 |
|---|---|---|---|---|
| 레이더 | range-bearing (3D) | TRACK | SINGLE_CLASS, 거친 수준 (Air Vehicle / Animal) | ENU 속도 |
| EO/IR (PTZ) | range-bearing (방위·고각만) | TRACKLET | MULTI_CLASS (UAV Rotary/Fixed, Bird…) | `look_at` 수용 |
| Passive RF | range-bearing (방위만) | NONE | RF Transmitter → UAV Transmitter | `signal[]` |
| 음향 | range-bearing (방위만) | NONE | SINGLE_CLASS, 낮은 신뢰도 | — |

---

## 참고 자료

- SAPIENT ICD v7.0: https://assets.publishing.service.gov.uk/media/6419a2068fa8f547c68029d3/SAPIENT_Interface_Control_Document_v7_FINAL__fixed2_.pdf
- dstl SAPIENT 개요: https://www.gov.uk/sapient , https://www.gov.uk/guidance/sapient-autonomous-sensor-system
- 2015 dstl 시험: https://www.gov.uk/government/news/plug-and-play-autonomous-sensors
- NATO TIE21: https://www.gov.uk/government/news/nato-trials-dstl-standard-for-counter-drone-systems
- NATO 채택 보도(TIE23): https://www.janes.com/defence-news/defence/latest/nato-to-adopt-sapient-as-c-uas-standard
- TIE24 (AEDP-4869): https://www.unmannedairspace.info/counter-uas-systems-and-policies/indra-demonstrates-crow-c-uas-at-nato-exercise/
- 로컬: `SAPIENT_MODULES/SAPIENT-Proto-Files/bsi_flex_335_v2_0/*.proto`, `SAPIENT_MODULES/BSI-Flex-335-v2-Test-Harness/SapientServices/Data/Validation/`
