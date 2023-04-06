---

database-plugin: basic

---

```yaml:dbfolder
name: new database
description: new description
columns:
  __file__:
    key: __file__
    id: __file__
    input: markdown
    label: File
    accessorKey: __file__
    isMetadata: true
    skipPersist: false
    isDragDisabled: false
    csvCandidate: true
    position: 1
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: true
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  rating:
    input: number
    accessorKey: rating
    key: rating
    id: rating
    label: Rating
    position: 5
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  Type:
    input: select
    accessorKey: Type
    key: Type
    id: Type
    label: Context
    position: 6
    skipPersist: false
    isHidden: false
    sortIndex: -1
    options:
      - { label: "Gesture", value: "Gesture", color: "hsl(304,100%,50%)"}
      - { label: "Impro", value: "Impro", color: "hsl(135,100%,50%)"}
      - { label: "Other", value: "Other", color: "hsl(32,100%,50%)"}
      - { label: "", value: "", color: "hsl(70, 95%, 90%)"}
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
      option_source: manual
  DateTime:
    input: calendar_time
    accessorKey: DateTime
    key: DateTime
    id: DateTime
    label: Created
    position: 7
    skipPersist: false
    isHidden: false
    sortIndex: 1
    width: 472
    isSorted: true
    isSortedDesc: true
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  Location:
    input: select
    accessorKey: Location
    key: Location
    id: Location
    label: Location
    position: 8
    skipPersist: false
    isHidden: false
    sortIndex: -1
    options:
      - { label: "Dock11 Eden", value: "Dock11 Eden", color: "hsl(167, 95%, 90%)"}
      - { label: "Dock11 Digital", value: "Dock11 Digital", color: "hsl(118, 95%, 90%)"}
      - { label: "Alte Hölle", value: "Alte Hölle", color: "hsl(112, 95%, 90%)"}
      - { label: "", value: "", color: "hsl(181, 95%, 90%)"}
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  Description:
    input: text
    accessorKey: Description
    key: Description
    id: Description
    label: Description
    position: 2
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  __inlinks__:
    key: __inlinks__
    id: __inlinks__
    input: inlinks
    label: Inlinks
    accessorKey: __inlinks__
    isMetadata: true
    isDragDisabled: false
    skipPersist: false
    csvCandidate: false
    position: 3
    isHidden: true
    sortIndex: -1
    width: 245
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  __outlinks__:
    key: __outlinks__
    id: __outlinks__
    input: outlinks
    label: Outlinks
    accessorKey: __outlinks__
    isMetadata: true
    isDragDisabled: false
    skipPersist: false
    csvCandidate: false
    position: 4
    isHidden: true
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  Targets:
    input: tags
    accessorKey: Targets
    key: Targets
    id: Targets
    label: Targets
    position: 9
    skipPersist: false
    isHidden: false
    sortIndex: -1
    options:
      - { label: "Back", value: "Back", color: "hsl(115, 95%, 90%)"}
      - { label: "LeftFoot", value: "LeftFoot", color: "hsl(226, 95%, 90%)"}
      - { label: "Head", value: "Head", color: "hsl(135, 95%, 90%)"}
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  Features:
    input: tags
    accessorKey: Features
    key: Features
    id: Features
    label: Features
    position: 11
    skipPersist: false
    isHidden: false
    sortIndex: -1
    options:
      - { label: "Moog01", value: "Moog01", color: "hsl(120, 95%, 90%)"}
      - { label: "Lead01", value: "Lead01", color: "hsl(46, 95%, 90%)"}
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  Performers:
    input: tags
    accessorKey: Performers
    key: Performers
    id: Performers
    label: Performers
    position: 12
    skipPersist: false
    isHidden: false
    sortIndex: -1
    width: 140
    options:
      - { label: "Kate", value: "Kate", color: "hsl(67, 95%, 90%)"}
      - { label: "Howool", value: "Howool", color: "hsl(83, 95%, 90%)"}
      - { label: "[[Kate]]", value: "[[Kate]]", color: "hsl(142, 95%, 90%)"}
      - { label: "[[Performers/Kate.md|Kate]]", value: "[[Performers/Kate.md|Kate]]", color: "hsl(97, 95%, 90%)"}
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  Batch_Size:
    input: number
    accessorKey: Batch_Size
    key: Batch_Size
    id: Batch_Size
    label: Batch Size
    position: 17
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  Type_1:
    input: select
    accessorKey: Type_1
    key: Type_1
    id: Type_1
    label: Type
    position: 13
    skipPersist: false
    isHidden: false
    sortIndex: -1
    options:
      - { label: "lstm", value: "lstm", color: "hsl(32, 95%, 90%)"}
      - { label: "linear regression", value: "linear regression", color: "hsl(59, 95%, 90%)"}
      - { label: "", value: "", color: "hsl(48, 95%, 90%)"}
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  mdn:
    input: checkbox
    accessorKey: mdn
    key: mdn
    id: mdn
    label: MDN
    position: 14
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  inputDim:
    input: number
    accessorKey: inputDim
    key: inputDim
    id: inputDim
    label: InputDim
    position: 15
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  outputDim:
    input: number
    accessorKey: outputDim
    key: outputDim
    id: outputDim
    label: OutputDim
    position: 16
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  Epochs:
    input: number
    accessorKey: Epochs
    key: Epochs
    id: Epochs
    label: Epochs
    position: 18
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  HiddenDim:
    input: number
    accessorKey: HiddenDim
    key: HiddenDim
    id: HiddenDim
    label: HiddenDim
    position: 19
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  LearningRate:
    input: number
    accessorKey: LearningRate
    key: LearningRate
    id: LearningRate
    label: LearningRate
    position: 20
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  TimeSteps:
    input: number
    accessorKey: TimeSteps
    key: TimeSteps
    id: TimeSteps
    label: TimeSteps
    position: 21
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  ComputationTime:
    input: number
    accessorKey: ComputationTime
    key: ComputationTime
    id: ComputationTime
    label: ComputationTime
    position: 22
    skipPersist: false
    isHidden: false
    sortIndex: -1
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: false
      task_hide_completed: true
      footer_type: none
      persist_changes: false
  Mod_Rec:
    input: relation
    accessorKey: Mod_Rec
    key: Mod_Rec
    id: Mod_Rec
    label: Mod_Rec
    position: 10
    skipPersist: false
    isHidden: false
    sortIndex: -1
    width: 233
    config:
      enable_media_view: true
      link_alias_enabled: true
      media_width: 100
      media_height: 100
      isInline: true
      task_hide_completed: true
      footer_type: none
      persist_changes: false
      related_note_path: Recordings/Recordings DB.md
      bidirectional_relation: true
      relation_color: hsl(0,8%,15%)
config:
  remove_field_when_delete_column: false
  cell_size: normal
  sticky_first_column: false
  group_folder_column: 
  remove_empty_folders: false
  automatically_group_files: false
  hoist_files_with_empty_attributes: true
  show_metadata_created: false
  show_metadata_modified: false
  show_metadata_tasks: false
  show_metadata_inlinks: true
  show_metadata_outlinks: true
  source_data: current_folder
  source_form_result: 
  source_destination_path: /
  row_templates_folder: /
  current_row_template: 
  pagination_size: 10
  font_size: 16
  enable_js_formulas: false
  formula_folder_path: /
  inline_default: false
  inline_new_position: last_field
  date_format: yyyy-MM-dd
  datetime_format: "yyyy-MM-dd HH:mm:ss"
  metadata_date_format: "yyyy-MM-dd HH:mm:ss"
  enable_footer: false
  implementation: default
filters:
  enabled: false
  conditions:
```