# sim_generation.py
import streamlit as st
import xml.sax.saxutils
import re
import os # For saving files to server/WSL environment
import xml.etree.ElementTree as ET # XMLパース用に追加
import io # ファイルI/O用に追加

# --- 0. アプリケーションの基本設定 ---
st.set_page_config(
    layout="wide",
    page_title="ISO 22166-202 SIM Generator",
    page_icon="🏭"
)

st.title("🏭 ISO 22166-202 Information Model (SIM) Generator")
st.caption("This GUI generates or imports a Software Information Model (SIM) XML compliant with the ISO 22166-202 standard.")

# --- 1. 定数と選択肢の定義 ---
ENUMS = {
    # Properties
    "NoBit": ["BIT16", "BIT32", "BIT64"],
    "OpTypes": ["PERIODIC", "EVENTDRIVEN", "NONRT"],
    "InstanceType": ["Singleton", "MultitonStatic", "MultitonComm"],
    "DependencyType": ["OWNER", "OWNED", "OWNEROWNED", "NONE"],
    # IDnType
    "L0_Category": {
        "HW/SW Module (00)": "00",
        "Hardware Module (01)": "01",
        "Software Module (10)": "10",
        "Development/Testing Tool (11)": "11"
    },
    "L1_Category": {
        "Planning (0000)": "0000",
        "Communication (0001)": "0001",
        "Interaction (0010)": "0010",
        "General Computing (0011)": "0011",
        "Orchestration/Management (0100)": "0100",
        "Sensing (0101)": "0101",
        "Actuating (0110)": "0110",
        "Reserved (0111-1111)": "0111"
    },
    "L2_Category": {
        "Planning": {
            "Motion planning (000000)": "000000",
            "Grasp planning (000001)": "000001",
            "Task planning (000010)": "000010"
        },
        "Communication": {
            "To/From Server (000000)": "000000",
            "To/From Other robot (000001)": "000001",
            "To/From Inner modules (000010)": "000010",
            "Clouds (000011)": "000011"
        },
        "Interaction": {
            "Speech recognition (000000)": "000000",
            "Speech generation (000001)": "000001",
            "Gesture recognition (000010)": "000010",
            "Structured dialog-based (000011)": "000011"
        },
        "General Computing": {
            "Localization (000000)": "000000",
            "Mapping (000001)": "000001",
            "Feature detection (000010)": "000010",
            "Generic data transformation (000011)": "000011",
            "Learning (000100)": "000100",
            "Control (000101)": "000101"
        },
        "Orchestration/Management": {
            "Orchestration service (000000)": "000000",
            "Monitoring service (000001)": "000001"
        },
        "Sensing": {
            "Perception service (000000)": "000000",
            "Recognition service (000001)": "000001",
            "Measurement service (000010)": "000010"
        },
        "Actuating": {
            "Electrical type (000100)": "000100",
            "Hydraulic type (000001)": "000001",
            "Pneumatic type (000010)": "000010",
            "Hybrid (Elec + Hyd) (000101)": "000101",
            "Hybrid (Elec + Pneu) (000110)": "000110",
            "Hybrid (Pneu + Hyd) (000011)": "000011",
            "Hybrid (Elec+Pneu + Hyd) (000111)": "000111"
        },
        "Reserved": {"Reserved (000000)": "000000"}
    },
    "InOutType": ["IN", "OUT", "INOUT"],
    "ExeStatus": ["CREATED", "IDLE", "EXECUTING", "DESTRUCTED", "ERROR"],
    "PhysicalVirtual": ["Physical", "Virtual"],
    "MOType": ["MANDATORY", "OPTIONAL"],
    "ReqProvType": ["REQUIRED", "PROVIDED"],
    "PLSILType": ["PL", "SIL", "BOTH", "NONE"],
    "SafetyLevelPL": ["n", "a", "b", "c", "d", "e"],
    "SafetyLevelSIL": ["0", "1", "2", "3", "4"],
    "SafetyType": ["ESTOP", "PSTOP", "LIMWS", "SRSC", "SRFC", "HCOLA", "STCON"],
    "SecurityLevelCyber": ["0", "1", "2", "3", "4"],
    "SecurityLevelPhysical": ["0", "LatchSensor", "LockwithKey", "LockwithActuator"],
    "SecurityType": [
        "HU_IA", "SD_IA", "ACNT_MGT", "ID_MGT", "AUTH_MGT", "WIRELEE_MGT", "PW_AUTH",
        "PK_CERT", "STR_PK_AUTH", "LOGIN_NO", "ACC_UNTRUST_NET", "AUTHORIZE", "WIRELESS_USE",
        "SESS_LOCK", "SESS_TERM", "SECC_CNTR", "AUDT_EVT", "TIMESTM", "NON_REP",
        "COMM_INTG", "PROT_MALI_CODE", "SECUR_VERIFY", "SW_INTGT", "INPUT_VALD",
        "DET_OUT", "ERR_HNDL", "SESS_INTGT", "INFO_CONFI", "INFO_PERS", "CRYTO",
        "RSTIC_FLOW", "DoS", "RESOU_MGT", "CNTR_RECOV_RECON"
    ]
}



# --- 2. セッションステートの初期化 ---

def initialize_state():
    """セッションステートをデフォルト値で初期化（またはリセット）する"""
    st.session_state.gen_info = {"moduleName": "", "description": "", "manufacturer": "", "examples": ""}
    st.session_state.idn_type = {"is_composite": False, "id_input_mode": "Guided Generator", "vid": "", "pid_has_sw": True, "pid_has_hw": False, "pid_safety": False, "pid_security": False, "pid_vendor": "", "rev": "", "serialNo": "00000000", "cat_l0": "Software Module (10)", "cat_l1": "General Computing (0011)", "cat_l2": "Localization (000000)", "cat_l3_hex": "00", "cat_l4_hex": "00", "iid": "00", "direct_mid": "", "direct_iid": "", "imv": "1.0", "swAspects": []}
    st.session_state.properties = {"os_type": "Ubuntu", "os_bit": "BIT64", "os_version": "24.04", "compiler_osName": "Ubuntu", "compiler_verRangeOS_min": "24.04", "compiler_verRangeOS_max": "24.04", "compiler_name": "GCC", "compiler_verRangeCompiler_min": "13", "compiler_verRangeCompiler_max": "Higher", "compiler_bitnCPUarch": "64, X86", "exeTypes": [{"id": 0, "opType": "PERIODIC", "hardRT": True, "timeConstraint": 10000.0, "priority": 10, "instanceType": "Singleton"}], "use_libs": False, "libs": [], "org_dependency": "OWNER", "org_members": [], "org_addInfo_use": False, "org_addInfo_nv": [], "use_custom_props": False, "custom_props": []}
    st.session_state.io_variables = {"inputs": [], "outputs": [], "inouts": []}
    st.session_state.status = {"executionStatus": "IDLE", "errorType": 0}
    st.session_state.services = {"profiles": []}
    st.session_state.infrastructure = {"database": [], "middleware": [], "comms": [], "use_addInfo": False, "addInfo_nv": []}
    st.session_state.safe_secure = {"overallValidSafetyLevelType": "NONE", "overallSafetyLevelPL": "n", "overallSafetyLevelSIL": "0", "overallPhySecurityLevel": "0", "overallCybSecurityLevel": "0", "inSafetyLevel": [], "inCybSecurityLevel": [], "use_addInfo": False, "addInfo_nv": []}
    st.session_state.modelling = {"simulationModel": []}
    st.session_state.executable_form = {"exeForm": [], "LibraryURL": []}
        
    # すべてのカウンターをリセット
    counters = ['exeType_counter', 'swAspect_counter', 'lib_counter', 'org_member_counter', 'org_nv_counter', 'custom_prop_counter', 'io_input_counter', 'io_output_counter', 'io_inout_counter', 'io_nv_counter', 'service_profile_counter', 'service_method_counter', 'service_arg_counter', 'service_nv_counter', 'infra_db_counter', 'infra_mw_counter', 'infra_comm_counter', 'infra_comm_proto_counter', 'infra_comm_net_counter', 'infra_comm_app_counter', 'infra_comm_nv_counter', 'infra_nv_counter', 'safe_safety_fn_counter', 'safe_cyber_fn_counter', 'safe_nv_counter', 'model_sim_counter', 'model_mdf_group_counter', 'model_mdf_item_counter', 'model_lib_counter', 'model_dyn_sw_counter', 'model_dyn_sw_shell_counter', 'model_dyn_sw_prop_counter', 'model_dyn_sw_nv_counter', 'model_nv_counter', 'exec_counter', 'exec_lib_counter', 'exec_shell_counter', 'exec_prop_counter', 'exec_nv_counter']
    for counter in counters:
        st.session_state[counter] = 0
    
    # exeTypeは最低1つ必要なのでカウンターを1にする
    st.session_state.exeType_counter = 1

    # XML出力とファイルパスもリセット
    st.session_state.xml_output = ""
    st.session_state.save_path_server = os.path.join(os.getcwd(), 'generated_models')
    st.session_state.filename_server = ''


# 初期化の呼び出し (初回ロード時)
if 'gen_info' not in st.session_state:
    initialize_state()


# --- 2.5. XMLパースロジック (新規追加) ---

# --- XML Parser Helpers ---
def find_text(element, tag, default=''):
    """指定されたタグのテキストを安全に取得します。"""
    try:
        found = element.find(tag)
        if found is not None and found.text is not None:
            return found.text
    except Exception:
        pass
    return default

def find_attr(element, tag, attr, default=''):
    """指定されたタグの属性を安全に取得します。"""
    try:
        found = element.find(tag)
        if found is not None:
            return found.attrib.get(attr, default)
    except Exception:
        pass
    return default

def parse_bool(s):
    """'true'/'false'の文字列をboolに変換します。"""
    if s is None: return False
    return s.lower() == 'true'

def parse_float(s, default=0.0):
    try:
        return float(s)
    except (ValueError, TypeError, AttributeError):
        return default

def parse_int(s, default=0):
    try:
        return int(s)
    except (ValueError, TypeError, AttributeError):
        return default

def parse_nv_list(element, counter_key):
    """additionalInfo (NVList) をパースします。"""
    nv_list = []
    add_info = element.find('additionalInfo')
    if add_info is not None:
        for nv in add_info.findall('nv'):
            new_id = st.session_state[counter_key]
            nv_list.append({
                "id": new_id, 
                "name": find_text(nv, 'Name'),
                "value": find_text(nv, 'Value')
            })
            st.session_state[counter_key] += 1
    return nv_list, (len(nv_list) > 0)

def parse_module_id(element):
    """moduleID をパースします。"""
    mod_id_elem = element.find('moduleID')
    if mod_id_elem is not None:
        return {
            "use_moduleID": True,
            "moduleID_mID": find_text(mod_id_elem, 'mID'),
            "moduleID_iID": find_text(mod_id_elem, 'iID')
        }
    return {"use_moduleID": False, "moduleID_mID": "", "moduleID_iID": ""}

def parse_property_list(element, counter_key):
    """<properties> または <Properties> タグ内の <Property> リストをパースします。"""
    prop_list = []
    # <properties> (Properties tab) or <Properties> (ExeForm)
    props_elem = element.find('properties')
    if props_elem is None:
        props_elem = element.find('Properties')
    
    if props_elem is not None:
        for prop in props_elem.findall('Property'):
            new_id = st.session_state[counter_key]
            val_from_text = find_text(prop, 'value', None)
            
            prop_data = {
                "id": new_id,
                "name": prop.attrib.get('name', ''),
                "type": prop.attrib.get('type', 'String'),
                "unit": prop.attrib.get('unit', ''),
                "description": prop.attrib.get('description', ''),
                "value": val_from_text if val_from_text is not None else prop.attrib.get('value', ''),
                "immutable": parse_bool(find_text(prop, 'immutable', 'false')) if val_from_text is not None else parse_bool(prop.attrib.get('immutable', 'false'))
            }
            prop_list.append(prop_data)
            st.session_state[counter_key] += 1
    return prop_list

def parse_exe_form(element, counter_key_prefix):
    """<exeForm> または <dynamicSW> タグをパースします (構造は同じ)"""
    new_id = st.session_state[f"{counter_key_prefix}_counter"]
    
    shell_list = []
    for cmd in element.findall('ShellCmd'):
        shell_id = st.session_state[f"{counter_key_prefix}_shell_counter"]
        if cmd.text is not None:
            shell_list.append({"id": shell_id, "value": cmd.text})
            st.session_state[f"{counter_key_prefix}_shell_counter"] += 1
            
    props_list = parse_property_list(element, f"{counter_key_prefix}_prop_counter")
    nv_list, use_addInfo = parse_nv_list(element, f"{counter_key_prefix}_nv_counter")
    
    exe_form_data = {
        "id": new_id,
        "exeFileURL": find_text(element, 'exeFileURL'),
        "shellCmd": shell_list,
        "properties": props_list,
        "use_addInfo": use_addInfo,
        "addInfo_nv": nv_list
    }
    
    st.session_state[f"{counter_key_prefix}_counter"] += 1
    return exe_form_data

def parse_and_load_xml(uploaded_file):
    """
    アップロードされたXMLファイルを解析し、st.session_stateにデータをロードする。
    """
    try:
        # 1. 状態をデフォルトにリセット (カウンター含む)
        initialize_state()
        
        # 2. ファイルを読み込み、XMLツリーを構築
        file_data = uploaded_file.getvalue()
        root = ET.fromstring(file_data)
        
        if root.tag != 'Module':
            st.error(f"Invalid XML: Root tag is not '<Module>'. Found '<{root.tag}>'.", icon="🚫")
            return

        # 3. 各セクションのパースとsession_stateへのロード
        
        # --- 1. GenInfo ---
        s_gen = st.session_state.gen_info
        s_gen['moduleName'] = find_text(root, 'moduleName', 'DefaultName')
        s_gen['description'] = find_text(root, 'description')
        s_gen['manufacturer'] = find_text(root, 'manufacturer', 'DefaultManufacturer')
        s_gen['examples'] = find_text(root, 'examples')
        
        # --- 2. IDnType ---
        s_id = st.session_state.idn_type
        idn_type_elem = root.find('idnType')
        if idn_type_elem is not None:
            s_id['is_composite'] = idn_type_elem.attrib.get('type') == 'Com'
            mod_id = idn_type_elem.find('moduleID')
            if mod_id is not None:
                # Guided Generatorへの逆マッピングはせず、Direct Inputとして扱う
                s_id['id_input_mode'] = "Direct Input"
                s_id['direct_mid'] = find_text(mod_id, 'mID')
                s_id['direct_iid'] = find_text(mod_id, 'iID')
            s_id['imv'] = find_text(idn_type_elem, 'informationModelVersion', '1.0')
            
            sw_aspects_elem = idn_type_elem.find('swAspects')
            if sw_aspects_elem is not None:
                s_id['swAspects'] = [] # リストをリセット
                for aspect in sw_aspects_elem.findall('moduleID'):
                    new_id = st.session_state.swAspect_counter
                    s_id['swAspects'].append({
                        "id": new_id,
                        "mID": find_text(aspect, 'mID'),
                        "iID": find_text(aspect, 'iID', '00')
                    })
                    st.session_state.swAspect_counter += 1

        # --- 3. Properties ---
        s_prop = st.session_state.properties
        props_elem = root.find('properties')
        if props_elem is not None:
            # osType
            os_elem = props_elem.find('osType')
            if os_elem is not None:
                s_prop['os_type'] = os_elem.attrib.get('type', 'Ubuntu')
                s_prop['os_bit'] = os_elem.attrib.get('bit', 'BIT64')
                s_prop['os_version'] = os_elem.attrib.get('version', '24.04')
            
            # compiler
            comp_elem = props_elem.find('compiler')
            if comp_elem is not None:
                s_prop['compiler_osName'] = find_text(comp_elem, 'osName', 'Ubuntu')
                s_prop['compiler_name'] = find_text(comp_elem, 'compilerName', 'GCC')
                s_prop['compiler_bitnCPUarch'] = find_text(comp_elem, 'bitnCPUarch', '64, X86')
                os_range = comp_elem.find('verRangeOS')
                if os_range is not None:
                    s_prop['compiler_verRangeOS_min'] = os_range.attrib.get('min', '24.04')
                    s_prop['compiler_verRangeOS_max'] = os_range.attrib.get('max', '24.04')
                comp_range = comp_elem.find('verRangeCompiler')
                if comp_range is not None:
                    s_prop['compiler_verRangeCompiler_min'] = comp_range.attrib.get('min', '13')
                    s_prop['compiler_verRangeCompiler_max'] = comp_range.attrib.get('max', 'Higher')
            
            # exeType
            exe_type_elem = props_elem.find('exeType')
            if exe_type_elem is not None:
                s_prop['exeTypes'] = [] # リストをリセット
                st.session_state.exeType_counter = 0 # カウンターリセット
                for item in exe_type_elem.findall('Item'):
                    new_id = st.session_state.exeType_counter
                    s_prop['exeTypes'].append({
                        "id": new_id,
                        "opType": find_text(item, 'opType', 'PERIODIC'),
                        "hardRT": parse_bool(find_text(item, 'hardRT', 'false')),
                        "timeConstraint": parse_float(find_text(item, 'timeConstraint', '1000.0')),
                        "priority": parse_int(find_text(item, 'priority', '50')),
                        "instanceType": find_text(item, 'instanceType', 'Singleton')
                    })
                    st.session_state.exeType_counter += 1
            
            # Libraries
            libs_elem = props_elem.find('Libraries')
            if libs_elem is not None:
                s_prop['use_libs'] = True
                s_prop['libs'] = [] # リストをリセット
                st.session_state.lib_counter = 0 # カウンターリセット
                for item in libs_elem.findall('Item'):
                    new_id = st.session_state.lib_counter
                    s_prop['libs'].append({
                        "id": new_id,
                        "name": item.attrib.get('name', ''),
                        "version": item.attrib.get('version', '')
                    })
                    st.session_state.lib_counter += 1
            
            # organization
            org_elem = props_elem.find('organization')
            if org_elem is not None:
                s_prop['org_dependency'] = find_text(org_elem, 'dependency', 'OWNER')
                s_prop['org_members'] = [] # リストをリセット
                st.session_state.org_member_counter = 0 # カウンターリセット
                for mem in org_elem.findall('member'):
                    new_id = st.session_state.org_member_counter
                    mem_id = mem.find('member')
                    s_prop['org_members'].append({
                        "id": new_id,
                        "mID": find_text(mem_id, 'mID') if mem_id is not None else "",
                        "iID": find_text(mem_id, 'iID', '00') if mem_id is not None else "00",
                        "dependency": find_text(mem, 'dependency', 'OWNED')
                    })
                    st.session_state.org_member_counter += 1
                
                nv_list, use_addInfo = parse_nv_list(org_elem, 'org_nv_counter')
                s_prop['org_addInfo_use'] = use_addInfo
                s_prop['org_addInfo_nv'] = nv_list

            # Custom Properties
            custom_props = parse_property_list(props_elem, 'custom_prop_counter')
            if custom_props:
                s_prop['use_custom_props'] = True
                s_prop['custom_props'] = custom_props

        # --- 4. IOVariables ---
        s_io = st.session_state.io_variables
        io_elem = root.find('ioVariables')
        if io_elem is not None:
            def parse_io_item_list(parent_elem, tag_name, list_key, counter_key):
                item_list_elem = io_elem.find(parent_elem)
                if item_list_elem is not None:
                    s_io[list_key] = [] # リストをリセット
                    st.session_state[counter_key] = 0 # カウンターリセット
                    for item in item_list_elem.findall(tag_name):
                        new_id = st.session_state[counter_key]
                        # IO変数内のNVListは共通のカウンターキーを使う
                        nv_list, use_addInfo = parse_nv_list(item, 'io_nv_counter')
                        mod_id_info = parse_module_id(item)
                        
                        s_io[list_key].append({
                            "id": new_id,
                            "name": item.attrib.get('name', ''),
                            "type": item.attrib.get('type', ''),
                            "value": find_text(item, 'value', item.attrib.get('value', '')),
                            "unit": find_text(item, 'unit', item.attrib.get('unit', '')),
                            "description": find_text(item, 'description', item.attrib.get('description', '')),
                            "className": item.attrib.get('className', ''),
                            **mod_id_info,
                            "use_addInfo": use_addInfo,
                            "addInfo_nv": nv_list
                        })
                        st.session_state[counter_key] += 1

            parse_io_item_list('Inputs', 'Input', 'inputs', 'io_input_counter')
            parse_io_item_list('Outputs', 'Output', 'outputs', 'io_output_counter')
            parse_io_item_list('InOuts', 'Inout', 'inouts', 'io_inout_counter')

        # --- 5. Status ---
        s_stat = st.session_state.status
        status_elem = root.find('Status')
        if status_elem is not None:
            s_stat['executionStatus'] = find_text(status_elem, 'executionStatus', 'IDLE')
            s_stat['errorType'] = parse_int(find_text(status_elem, 'errorType', '0'))

        # --- 6. Services ---
        s_serv = st.session_state.services
        serv_elem = root.find('services')
        if serv_elem is not None:
            s_serv['profiles'] = [] # リストをリセット
            st.session_state.service_profile_counter = 0 # カウンターリセット
            
            for idl in serv_elem.findall('ServiceIDL'):
                new_id = st.session_state.service_profile_counter
                s_serv['profiles'].append({
                    "id": new_id,
                    "profile_id": f"IDL_Service_{new_id}",
                    "ifURL": idl.text if idl.text is not None else "",
                    "methodList_type": "IDL",
                    "pvType": "Virtual", "moType": "MANDATORY",
                    "use_moduleID": False, "moduleID_mID": "", "moduleID_iID": "",
                    "use_addInfo": False, "addInfo_nv": [], "methodList": []
                })
                st.session_state.service_profile_counter += 1
            
            for xml_serv in serv_elem.findall('ServiceXML'):
                new_id = st.session_state.service_profile_counter
                # サービス関連のNVListは共通のカウンターキーを使う
                nv_list, use_addInfo = parse_nv_list(xml_serv, 'service_nv_counter')
                mod_id_info = parse_module_id(xml_serv)
                
                profile = {
                    "id": new_id,
                    "profile_id": find_text(xml_serv, 'ID', f'XML_Service_{new_id}'),
                    "ifURL": "",
                    "methodList_type": "XML",
                    "pvType": find_text(xml_serv, 'pvType', 'Virtual'),
                    "moType": find_text(xml_serv, 'moType', 'MANDATORY'),
                    **mod_id_info,
                    "use_addInfo": use_addInfo,
                    "addInfo_nv": nv_list,
                    "methodList": []
                }
                
                method_list_elem = xml_serv.find('methodList')
                if method_list_elem is not None:
                    for method_elem in method_list_elem.findall('Method'):
                        new_m_id = st.session_state.service_method_counter
                        m_nv_list, m_use_addInfo = parse_nv_list(method_elem, 'service_nv_counter')
                        
                        method = {
                            "id": new_m_id,
                            "methodName": find_text(method_elem, 'methodName'),
                            "retType": find_text(method_elem, 'retType', 'void'),
                            "moType": find_text(method_elem, 'moType', 'MANDATORY'),
                            "reqProvType": find_text(method_elem, 'reqProvType', 'PROVIDED'),
                            "use_addInfo": m_use_addInfo,
                            "addInfo_nv": m_nv_list,
                            "argType": []
                        }
                        
                        for arg_elem in method_elem.findall('argType'):
                            new_a_id = st.session_state.service_arg_counter
                            a_nv_list, a_use_addInfo = parse_nv_list(arg_elem, 'service_nv_counter')
                            method['argType'].append({
                                "id": new_a_id,
                                "type": find_text(arg_elem, 'type', arg_elem.attrib.get('type', 'String')),
                                "valueName": find_text(arg_elem, 'valueName', arg_elem.attrib.get('valueName', '')),
                                "inout": find_text(arg_elem, 'inout', arg_elem.attrib.get('inout', 'IN')),
                                "use_addInfo": a_use_addInfo,
                                "addInfo_nv": a_nv_list
                            })
                            st.session_state.service_arg_counter += 1
                            
                        profile['methodList'].append(method)
                        st.session_state.service_method_counter += 1
                        
                s_serv['profiles'].append(profile)
                st.session_state.service_profile_counter += 1

        # --- 7. Infrastructure ---
        s_infra = st.session_state.infrastructure
        infra_elem = root.find('infra')
        if infra_elem is not None:
            def parse_infra_type_list(parent_tag, list_key, counter_key):
                parent_elem = infra_elem.find(parent_tag)
                if parent_elem is not None:
                    s_infra[list_key] = [] # リストをリセット
                    st.session_state[counter_key] = 0 # カウンターリセット
                    for item in parent_elem.findall('InfraType'):
                        new_id = st.session_state[counter_key]
                        ver = item.find('Version')
                        s_infra[list_key].append({
                            "id": new_id,
                            "name": find_text(item, 'Name'),
                            "version_min": ver.attrib.get('min', '') if ver is not None else "",
                            "version_max": ver.attrib.get('max', '') if ver is not None else ""
                        })
                        st.session_state[counter_key] += 1
            
            parse_infra_type_list('Database', 'database', 'infra_db_counter')
            parse_infra_type_list('Middleware', 'middleware', 'infra_mw_counter')
            
            comms_elem = infra_elem.find('comms')
            if comms_elem is not None:
                s_infra['comms'] = [] # リストをリセット
                st.session_state.infra_comm_counter = 0 # カウンターリセット
                for comm_elem in comms_elem.findall('Communication'):
                    new_id = st.session_state.infra_comm_counter
                    comm = {
                        "id": new_id,
                        "mostTopProtocol": [],
                        "underlyingProtocol": {}
                    }
                    
                    # mostTopProtocol
                    top_proto_elem = comm_elem.find('mostTopProtocol')
                    if top_proto_elem is not None:
                        for item in top_proto_elem.findall('InfraType'):
                            new_p_id = st.session_state.infra_comm_proto_counter
                            ver = item.find('Version')
                            comm['mostTopProtocol'].append({
                                "id": new_p_id,
                                "name": find_text(item, 'Name'),
                                "version_min": ver.attrib.get('min', '') if ver is not None else "",
                                "version_max": ver.attrib.get('max', '') if ver is not None else ""
                            })
                            st.session_state.infra_comm_proto_counter += 1
                    
                    # underlyingProtocol
                    dbus_elem = comm_elem.find('underlyingProtocol/Databus')
                    if dbus_elem is not None:
                        nv_list, use_addInfo = parse_nv_list(dbus_elem, 'infra_comm_nv_counter')
                        dbus = {
                            "connectionType": find_text(dbus_elem, 'ConnectorType', 'RJ45'),
                            "typePhyMac": find_text(dbus_elem, 'TypePhyMac', 'Ethernet'),
                            "speed": parse_float(find_attr(dbus_elem, 'Speed', 'value', '100000.0')),
                            "typeNetTrans": [],
                            "typeApp": [],
                            "use_addInfo": use_addInfo,
                            "addInfo_nv": nv_list
                        }
                        
                        net_elem = dbus_elem.find('TypeNetTrans')
                        if net_elem is not None:
                            for item in net_elem.findall('Item'):
                                new_n_id = st.session_state.infra_comm_net_counter
                                dbus['typeNetTrans'].append({"id": new_n_id, "value": item.text or ""})
                                st.session_state.infra_comm_net_counter += 1
                        
                        app_elem = dbus_elem.find('TypeApp')
                        if app_elem is not None:
                            for item in app_elem.findall('Item'):
                                new_a_id = st.session_state.infra_comm_app_counter
                                dbus['typeApp'].append({"id": new_a_id, "value": item.text or ""})
                                st.session_state.infra_comm_app_counter += 1
                        
                        comm['underlyingProtocol'] = dbus
                    
                    s_infra['comms'].append(comm)
                    st.session_state.infra_comm_counter += 1

            nv_list, use_addInfo = parse_nv_list(infra_elem, 'infra_nv_counter')
            s_infra['use_addInfo'] = use_addInfo
            s_infra['addInfo_nv'] = nv_list
            
        # --- 8. SafeSecure ---
        s_safe = st.session_state.safe_secure
        safe_elem = root.find('safeSecure')
        if safe_elem is not None:
            safety_elem = safe_elem.find('Safety')
            if safety_elem is not None:
                # overall
                overalls = safety_elem.findall('overall')
                if not overalls:
                    s_safe['overallValidSafetyLevelType'] = 'NONE'
                else:
                    has_pl = False
                    has_sil = False
                    for ov in overalls:
                        mode = ov.attrib.get('mode', '')
                        if mode == 'PL':
                            s_safe['overallSafetyLevelPL'] = ov.text or 'n'
                            has_pl = True
                        elif mode == 'SIL':
                            s_safe['overallSafetyLevelSIL'] = ov.text or '0'
                            has_sil = True
                    if has_pl and has_sil: s_safe['overallValidSafetyLevelType'] = 'BOTH'
                    elif has_pl: s_safe['overallValidSafetyLevelType'] = 'PL'
                    elif has_sil: s_safe['overallValidSafetyLevelType'] = 'SIL'
                
                # inSafetyLevel
                s_safe['inSafetyLevel'] = [] # リストをリセット
                st.session_state.safe_safety_fn_counter = 0 # カウンターリセット
                for item in safety_elem.findall('inSafetyLevel'):
                    new_id = st.session_state.safe_safety_fn_counter
                    s_safe['inSafetyLevel'].append({
                        "id": new_id,
                        "safetyFunctionType": item.attrib.get('safetyFunctionType', 'ESTOP'),
                        "validSafetyLevelType": item.attrib.get('validSafetyLevelType', 'NONE'),
                        "eachSafetyLevelPL": item.attrib.get('eachSafetyLevelPL', 'n'),
                        "eachSafetyLevelSIL": item.attrib.get('eachSafetyLevelSIL', '0')
                    })
                    st.session_state.safe_safety_fn_counter += 1
            
            security_elem = safe_elem.find('Security/CyberSecurity')
            if security_elem is not None:
                s_safe['overallCybSecurityLevel'] = find_text(security_elem, 'OverallCybSecurityLevel', '0')
                s_safe['inCybSecurityLevel'] = [] # リストをリセット
                st.session_state.safe_cyber_fn_counter = 0 # カウンターリセット
                for item in security_elem.findall('inCybSecurityLevel'):
                    new_id = st.session_state.safe_cyber_fn_counter
                    s_safe['inCybSecurityLevel'].append({
                        "id": new_id,
                        "securityType": item.attrib.get('type', 'HU_IA'),
                        "eachSecurityLevel": item.attrib.get('value', '0')
                    })
                    st.session_state.safe_cyber_fn_counter += 1
            
            # (Note: PhysicalSecurity is not in 202 SIM schema, only 201)
            
            nv_list, use_addInfo = parse_nv_list(safe_elem, 'safe_nv_counter')
            s_safe['use_addInfo'] = use_addInfo
            s_safe['addInfo_nv'] = nv_list

        # --- 9. Modelling ---
        s_model = st.session_state.modelling
        model_elem = root.find('modelling')
        if model_elem is not None:
            s_model['simulationModel'] = [] # リストをリセット
            st.session_state.model_sim_counter = 0 # カウンターリセット
            for sim_elem in model_elem.findall('simulationModel'):
                new_id = st.session_state.model_sim_counter
                nv_list, use_addInfo = parse_nv_list(sim_elem, 'model_nv_counter')
                
                sim = {
                    "id": new_id,
                    "simulator": find_text(sim_elem, 'simulator', 'Gazebo'),
                    "mdf": [],
                    "libraries": [],
                    "dynamicSW": [],
                    "use_addInfo": use_addInfo,
                    "addInfo_nv": nv_list
                }

                # mdf
                for mdf_group_elem in sim_elem.findall('mdf'):
                    new_g_id = st.session_state.model_mdf_group_counter
                    mdf_group = {
                        "id": new_g_id,
                        "type": mdf_group_elem.attrib.get('type', 'URDF'),
                        "items": []
                    }
                    for item_elem in mdf_group_elem.findall('Item'):
                        new_i_id = st.session_state.model_mdf_item_counter
                        mdf_group['items'].append({"id": new_i_id, "value": item_elem.text or ""})
                        st.session_state.model_mdf_item_counter += 1
                    sim['mdf'].append(mdf_group)
                    st.session_state.model_mdf_group_counter += 1
                
                # libraries
                for lib_elem in sim_elem.findall('libraries'):
                    new_l_id = st.session_state.model_lib_counter
                    sim['libraries'].append({"id": new_l_id, "value": lib_elem.text or ""})
                    st.session_state.model_lib_counter += 1
                
                # dynamicSW
                for dyn_sw_elem in sim_elem.findall('dynamicSW'):
                    # dynamicSW用のカウンタープレフィックスを使用
                    sim['dynamicSW'].append(parse_exe_form(dyn_sw_elem, 'model_dyn_sw'))
                
                s_model['simulationModel'].append(sim)
                st.session_state.model_sim_counter += 1

        # --- 10. ExecutableForm ---
        s_exec = st.session_state.executable_form
        exec_elem = root.find('execForm')
        if exec_elem is not None:
            s_exec['LibraryURL'] = [] # リストをリセット
            st.session_state.exec_lib_counter = 0 # カウンターリセット
            for lib_url in exec_elem.findall('libraryURL'):
                new_id = st.session_state.exec_lib_counter
                s_exec['LibraryURL'].append({"id": new_id, "value": lib_url.text or ""})
                st.session_state.exec_lib_counter += 1
            
            s_exec['exeForm'] = [] # リストをリセット
            st.session_state.exec_form_counter = 0 # カウンターリセット
            for exe_form_elem in exec_elem.findall('exeForm'):
                # exeForm用のカウンタープレフィックスを使用
                s_exec['exeForm'].append(parse_exe_form(exe_form_elem, 'exec'))

        # --- 完了 ---
        st.success(f"Successfully loaded data from '{uploaded_file.name}'!", icon="✅")
        # プレビュー用に読み込んだXMLをそのまま表示
        st.session_state.xml_output = file_data.decode('utf-8') 
        # ファイル名を自動設定
        st.session_state.filename_server = uploaded_file.name

    except ET.ParseError as e:
        st.error(f"XML Parse Error: Failed to parse the file. Check if it is valid XML.\nDetails: {e}", icon="🔥")
        initialize_state() # エラー時は状態をリセット
    except Exception as e:
        st.error(f"An unexpected error occurred during loading: {e}", icon="🔥")
        st.exception(e) # 詳細なトレースバック
        initialize_state() # エラー時は状態をリセット


# --- 3. 動的リスト管理関数 (多重度N用) ---

# --- Helper function for simple string list (e.g., shellCmd, LibraryURL) ---
def add_string_item(str_list, counter_key):
    new_id = st.session_state[counter_key]
    str_list.append({"id": new_id, "value": ""})
    st.session_state[counter_key] += 1

def del_string_item(str_list, item_id):
    str_list[:] = [item for item in str_list if item['id'] != item_id]

# --- Helper function for NVList (Name-Value) ---
def add_nv_item(list_key, counter_key):
    new_id = st.session_state[counter_key]
    list_key.append({"id": new_id, "name": "", "value": ""})
    st.session_state[counter_key] += 1

def del_nv_item(list_key, item_id):
    list_key[:] = [item for item in list_key if item['id'] != item_id]

# --- Helper function for ModuleID (used conditionally) ---
def render_module_id_input(state, key_prefix):
    state['use_moduleID'] = st.checkbox("Define ModuleID (Optional, for Composite)", value=state.get('use_moduleID', False), key=f"{key_prefix}_use_modid")
    if state['use_moduleID']:
        c1, c2 = st.columns(2)
        state['moduleID_mID'] = c1.text_input("ModuleID mID (62 hex)*", value=state.get('moduleID_mID', ''), key=f"{key_prefix}_mid", max_chars=62)
        state['moduleID_iID'] = c2.text_input("ModuleID iID (2 hex)*", value=state.get('moduleID_iID', ''), key=f"{key_prefix}_iid", max_chars=2)

# --- Helper function for AdditionalInfo (NVList) ---
def render_additional_info_editor(state, key_prefix, counter_key):
    state['use_addInfo'] = st.checkbox("Define Additional Info (Optional)", value=state.get('use_addInfo', False), key=f"{key_prefix}_use_nv")
    if state['use_addInfo']:
        if 'addInfo_nv' not in state: state['addInfo_nv'] = []
        
        for i in range(len(state['addInfo_nv'])):
            nv_item = state['addInfo_nv'][i]
            nv_id = nv_item['id']
            c1, c2, c3 = st.columns([2, 2, 1])
            nv_item['name'] = c1.text_input("NV Name*", value=nv_item['name'], key=f"{key_prefix}_nv_name_{nv_id}")
            nv_item['value'] = c2.text_input("NV Value*", value=nv_item['value'], key=f"{key_prefix}_nv_val_{nv_id}")
            c3.button("Del NV", on_click=del_nv_item, args=(state['addInfo_nv'], nv_id), key=f"{key_prefix}_nv_del_{nv_id}", use_container_width=True)
        
        st.button("➕ Add NV Pair", on_click=add_nv_item, args=(state['addInfo_nv'], counter_key), key=f"{key_prefix}_nv_add", use_container_width=True)

# --- Helper function for Property (Table 17) ---
def add_property_item(prop_list, counter_key):
    new_id = st.session_state[counter_key]
    prop_list.append({
        "id": new_id, "name": "", "type": "String", "unit": "", "description": "", "value": "", "immutable": False
    })
    st.session_state[counter_key] += 1
def del_property_item(prop_list, item_id):
    prop_list[:] = [item for item in prop_list if item['id'] != item_id]
def render_property_editor(prop_list, counter_key, key_prefix):
    for i in range(len(prop_list)):
        prop = prop_list[i]
        prop_id = prop['id']
        with st.container(border=True):
            st.markdown(f"**Property {i+1}**")
            c1, c2, c3 = st.columns([2, 1, 1])
            prop['name'] = c1.text_input("Name*", value=prop['name'], key=f"{key_prefix}_prop_name_{prop_id}")
            prop['type'] = c2.text_input("Type (String, Float...)*", value=prop['type'], key=f"{key_prefix}_prop_type_{prop_id}")
            prop['value'] = c3.text_input("Value*", value=prop['value'], key=f"{key_prefix}_prop_val_{prop_id}")
            c1, c2, c3, c4 = st.columns([1, 2, 1, 1])
            prop['unit'] = c1.text_input("Unit (Optional)", value=prop['unit'], key=f"{key_prefix}_prop_unit_{prop_id}")
            prop['description'] = c2.text_input("Description (Optional)", value=prop['description'], key=f"{key_prefix}_prop_desc_{prop_id}")
            prop['immutable'] = c3.checkbox("Immutable*", value=prop['immutable'], key=f"{key_prefix}_prop_immut_{prop_id}", help="True: Not configurable (M, 1)")
            c4.button("Delete Property", on_click=del_property_item, args=(prop_list, prop_id), key=f"{key_prefix}_prop_del_{prop_id}", use_container_width=True)
    st.button("➕ Add Property", on_click=add_property_item, args=(prop_list, counter_key), key=f"{key_prefix}_prop_add", use_container_width=True)

# --- Helper function for ExeForm (Table 34) ---
# (カウンターキーのプレフィックスを引数で受け取るように変更)
def add_exe_form_item(exe_form_list, counter_key_prefix):
    counter_key = f"{counter_key_prefix}_counter"
    new_id = st.session_state[counter_key]
    exe_form_list.append({
        "id": new_id, "exeFileURL": "", "shellCmd": [], "properties": [],
        "use_addInfo": False, "addInfo_nv": []
    })
    st.session_state[counter_key] += 1

def del_exe_form_item(exe_form_list, item_id):
    exe_form_list[:] = [item for item in exe_form_list if item['id'] != item_id]

# (カウンターキーのプレフィックスを引数で受け取るように変更)
def render_exe_form_editor(exe_form_list, counter_key_prefix, key_prefix):
    for i in range(len(exe_form_list)):
        exe_form = exe_form_list[i]
        exe_id = exe_form['id']
        with st.container(border=True):
            st.markdown(f"**Executable Form {i+1}**")
            exe_form['exeFileURL'] = st.text_input("Executable File URL*", value=exe_form['exeFileURL'], key=f"{key_prefix}_exe_url_{exe_id}")
            
            # shellCmd (O, N, String)
            st.markdown("**Shell Commands (shellCmd)** (O, N)")
            if 'shellCmd' not in exe_form: exe_form['shellCmd'] = [] # パース後用
            for j in range(len(exe_form['shellCmd'])):
                cmd_item = exe_form['shellCmd'][j]
                cmd_id = cmd_item['id']
                c1, c2 = st.columns([4, 1])
                cmd_item['value'] = c1.text_input(f"Command {j+1}", value=cmd_item['value'], key=f"{key_prefix}_exe_cmd_val_{exe_id}_{cmd_id}")
                c2.button("Del Cmd", on_click=del_string_item, args=(exe_form['shellCmd'], cmd_id), key=f"{key_prefix}_exe_cmd_del_{exe_id}_{cmd_id}", use_container_width=True)
            st.button("➕ Add Shell Command", on_click=add_string_item, args=(exe_form['shellCmd'], f'{counter_key_prefix}_shell_counter'), key=f"{key_prefix}_exe_cmd_add_{exe_id}", use_container_width=True)

            # properties (O, N, Property)
            st.markdown("**Properties (properties)** (O, N)")
            if 'properties' not in exe_form: exe_form['properties'] = [] # パース後用
            render_property_editor(exe_form['properties'], f'{counter_key_prefix}_prop_counter', f"{key_prefix}_exe_prop_{exe_id}")
            
            # additionalInfo (O, 1, NVList)
            render_additional_info_editor(exe_form, f"{key_prefix}_exe_nv_{exe_id}", f'{counter_key_prefix}_nv_counter')
            
            st.button("Delete Executable Form", on_click=del_exe_form_item, args=(exe_form_list, exe_id), key=f"{key_prefix}_exe_del_{exe_id}", use_container_width=True)
    st.button("➕ Add Executable Form", on_click=add_exe_form_item, args=(exe_form_list, counter_key_prefix), key=f"{key_prefix}_exe_add", use_container_width=True)


# --- IDnType ---
def add_sw_aspect():
    new_id = st.session_state.swAspect_counter
    st.session_state.idn_type['swAspects'].append({"id": new_id, "mID": "", "iID": "00"})
    st.session_state.swAspect_counter += 1
def del_sw_aspect(item_id):
    st.session_state.idn_type['swAspects'] = [item for item in st.session_state.idn_type['swAspects'] if item['id'] != item_id]

# --- Properties ---
def add_exeType():
    new_id = st.session_state.exeType_counter
    st.session_state.properties['exeTypes'].append({
        "id": new_id, "opType": "PERIODIC", "hardRT": False, "timeConstraint": 1000.0,
        "priority": 50, "instanceType": "Singleton"
    })
    st.session_state.exeType_counter += 1
def del_exeType(item_id):
    if len(st.session_state.properties['exeTypes']) > 1:
        st.session_state.properties['exeTypes'] = [item for item in st.session_state.properties['exeTypes'] if item['id'] != item_id]
    else:
        st.toast("Error: At least one exeType is Mandatory (M).", icon="🚫")
def add_lib():
    new_id = st.session_state.lib_counter
    st.session_state.properties['libs'].append({"id": new_id, "name": "", "version": ""})
    st.session_state.lib_counter += 1
def del_lib(item_id):
    st.session_state.properties['libs'] = [item for item in st.session_state.properties['libs'] if item['id'] != item_id]
def add_org_member():
    new_id = st.session_state.org_member_counter
    st.session_state.properties['org_members'].append({"id": new_id, "mID": "", "iID": "00", "dependency": "OWNED"})
    st.session_state.org_member_counter += 1
def del_org_member(item_id):
    st.session_state.properties['org_members'] = [item for item in st.session_state.properties['org_members'] if item['id'] != item_id]

# --- IOVariables ---
def add_io_variable(var_type):
    counters = {"inputs": 'io_input_counter', "outputs": 'io_output_counter', "inouts": 'io_inout_counter'}
    new_id = st.session_state[counters[var_type]]
    st.session_state.io_variables[var_type].append({
        "id": new_id, "name": "", "type": "", "value": "", "unit": "", "description": "",
        "className": "", "use_moduleID": False, "moduleID_mID": "", "moduleID_iID": "",
        "use_addInfo": False, "addInfo_nv": []
    })
    st.session_state[counters[var_type]] += 1
def del_io_variable(var_type, item_id):
    st.session_state.io_variables[var_type][:] = [item for item in st.session_state.io_variables[var_type] if item['id'] != item_id]

# --- Services ---
def add_service_profile():
    new_id = st.session_state.service_profile_counter
    st.session_state.services['profiles'].append({
        "id": new_id, "profile_id": "", "ifURL": "", "methodList_type": "XML",
        "pvType": "Virtual", "moType": "MANDATORY", "use_moduleID": False,
        "moduleID_mID": "", "moduleID_iID": "", "use_addInfo": False, "addInfo_nv": [], "methodList": []
    })
    st.session_state.service_profile_counter += 1
def del_service_profile(item_id):
    st.session_state.services['profiles'][:] = [p for p in st.session_state.services['profiles'] if p['id'] != item_id]
def add_service_method(profile):
    new_id = st.session_state.service_method_counter
    profile['methodList'].append({
        "id": new_id, "methodName": "", "retType": "void", "moType": "MANDATORY",
        "reqProvType": "PROVIDED", "use_addInfo": False, "addInfo_nv": [], "argType": []
    })
    st.session_state.service_method_counter += 1
def del_service_method(profile, item_id):
    profile['methodList'][:] = [m for m in profile['methodList'] if m['id'] != item_id]
def add_service_arg(method):
    new_id = st.session_state.service_arg_counter
    method['argType'].append({
        "id": new_id, "type": "String", "valueName": "", "inout": "IN",
        "use_addInfo": False, "addInfo_nv": []
    })
    st.session_state.service_arg_counter += 1
def del_service_arg(method, item_id):
    method['argType'][:] = [a for a in method['argType'] if a['id'] != item_id]

# --- Infrastructure ---
def add_infra_item(list_key, counter_key):
    new_id = st.session_state[counter_key]
    st.session_state.infrastructure[list_key].append({"id": new_id, "name": "", "version_min": "", "version_max": ""})
    st.session_state[counter_key] += 1
def del_infra_item(list_key, item_id):
    st.session_state.infrastructure[list_key][:] = [item for item in st.session_state.infrastructure[list_key] if item['id'] != item_id]
def add_infra_comm():
    new_id = st.session_state.infra_comm_counter
    st.session_state.infrastructure['comms'].append({
        "id": new_id, "mostTopProtocol": [],
        "underlyingProtocol": {"connectionType": "RJ45", "typePhyMac": "Ethernet", "speed": 100000.0, "typeNetTrans": [], "typeApp": [], "use_addInfo": False, "addInfo_nv": []}
    })
    st.session_state.infra_comm_counter += 1
    # 新規追加時、デフォルトのプロトコルも1つ追加
    add_infra_comm_proto(st.session_state.infrastructure['comms'][-1])
def del_infra_comm(item_id):
    st.session_state.infrastructure['comms'][:] = [c for c in st.session_state.infrastructure['comms'] if c['id'] != item_id]
def add_infra_comm_proto(comm_item):
    new_id = st.session_state.infra_comm_proto_counter
    comm_item['mostTopProtocol'].append({"id": new_id, "name": "", "version_min": "", "version_max": ""})
    st.session_state.infra_comm_proto_counter += 1
def del_infra_comm_proto(comm_item, item_id):
    # (削除ロジックは変更なし)
    if len(comm_item['mostTopProtocol']) > 1:
        comm_item['mostTopProtocol'][:] = [p for p in comm_item['mostTopProtocol'] if p['id'] != item_id]
    else:
        st.toast("Error: At least one 'mostTopProtocol' is Mandatory (M).", icon="🚫")

# --- SafeSecure (NEW) ---
def add_safety_function():
    new_id = st.session_state.safe_safety_fn_counter
    st.session_state.safe_secure['inSafetyLevel'].append({
        "id": new_id, "safetyFunctionType": "ESTOP", "validSafetyLevelType": "PL",
        "eachSafetyLevelPL": "n", "eachSafetyLevelSIL": "0"
    })
    st.session_state.safe_safety_fn_counter += 1
def del_safety_function(item_id):
    st.session_state.safe_secure['inSafetyLevel'][:] = [f for f in st.session_state.safe_secure['inSafetyLevel'] if f['id'] != item_id]
def add_cyber_security_function():
    new_id = st.session_state.safe_cyber_fn_counter
    st.session_state.safe_secure['inCybSecurityLevel'].append({
        "id": new_id, "securityType": "HU_IA", "eachSecurityLevel": "0"
    })
    st.session_state.safe_cyber_fn_counter += 1
def del_cyber_security_function(item_id):
    st.session_state.safe_secure['inCybSecurityLevel'][:] = [f for f in st.session_state.safe_secure['inCybSecurityLevel'] if f['id'] != item_id]

# --- Modelling (NEW) ---
def add_simulation_model():
    new_id = st.session_state.model_sim_counter
    st.session_state.modelling['simulationModel'].append({
        "id": new_id, "simulator": "Gazebo", "mdf": [], "libraries": [], "dynamicSW": [],
        "use_addInfo": False, "addInfo_nv": []
    })
    st.session_state.model_sim_counter += 1
    # 新規追加時、デフォルトのMDFグループも1つ追加
    add_mdf_group(st.session_state.modelling['simulationModel'][-1])
def del_simulation_model(item_id):
    st.session_state.modelling['simulationModel'][:] = [m for m in st.session_state.modelling['simulationModel'] if m['id'] != item_id]
def add_mdf_group(sim_model):
    new_id = st.session_state.model_mdf_group_counter
    mdf_group = {"id": new_id, "type": "URDF", "items": []}
    sim_model['mdf'].append(mdf_group)
    st.session_state.model_mdf_group_counter += 1
    # 新規追加時、デフォルトのMDFアイテムも1つ追加
    add_mdf_item(mdf_group)
def del_mdf_group(sim_model, item_id):
    # (削除ロジックは変更なし)
    if len(sim_model['mdf']) > 1:
        sim_model['mdf'][:] = [g for g in sim_model['mdf'] if g['id'] != item_id]
    else:
        st.toast("Error: At least one 'mdf' group is Mandatory (M).", icon="🚫")
def add_mdf_item(mdf_group):
    new_id = st.session_state.model_mdf_item_counter
    mdf_group['items'].append({"id": new_id, "value": ""})
    st.session_state.model_mdf_item_counter += 1
def del_mdf_item(mdf_group, item_id):
    # (削除ロジックは変更なし)
    if len(mdf_group['items']) > 1:
        mdf_group['items'][:] = [i for i in mdf_group['items'] if i['id'] != item_id]
    else:
        st.toast("Error: At least one 'mdf' item (path) is Mandatory (M).", icon="🚫")

# --- ExecutableForm (NEW) ---
# (render_exe_form_editor と add_exe_form_item をプレフィックスベースに変更したため、ここは変更なし)


# --- 4. XML生成ロジック ---

# --- Server Save Logic ---
def save_xml_to_server(xml_content, file_path, filename):
    """
    WSL/Ubuntu環境の指定されたフォルダにXMLファイルを保存する
    """
    if not os.path.isdir(file_path):
        os.makedirs(file_path, exist_ok=True) # フォルダがなければ作成

    full_path = os.path.join(file_path, filename)

    try:
        with open(full_path, 'w', encoding='utf-8') as f:
            f.write(xml_content)
        return True, full_path
    except Exception as e:
        return False, str(e)


def validate_and_generate_xml():
    # --- XML生成ロジックに必要なヘルパー定義 (再掲) ---
    def esc(s): return xml.sax.saxutils.escape(str(s))
    def validate_hex(s, length, field_name):
        if not s or len(s) != length or not re.fullmatch(r"[0-9a-fA-F]+", s):
            raise ValueError(f"{field_name}: Must be {length} hexadecimal characters. Value: '{s}'")
        return s.lower()
    
    def generate_nv_list_xml(nv_list, indent="      "):
        if not nv_list: return ""
        xml = [f'{indent}<additionalInfo>']
        for nv in nv_list:
            if nv['name'] or nv['value']:
                xml.append(f'{indent}  <nv>')
                xml.append(f'{indent}    <Name>{esc(nv["name"])}</Name>')
                xml.append(f'{indent}    <Value>{esc(nv["value"])}</Value>')
                xml.append(f'{indent}  </nv>')
        xml.append(f'{indent}</additionalInfo>')
        return "\n".join(xml)

    def generate_module_id_xml(state_dict, indent="    "):
        if not state_dict.get('use_moduleID', False): return ""
        mid = validate_hex(state_dict.get('moduleID_mID', ''), 62, "ModuleID mID")
        iid = validate_hex(state_dict.get('moduleID_iID', ''), 2, "ModuleID iID")
        xml = [f'{indent}<moduleID>', f'{indent}  <mID>{esc(mid)}</mID>', f'{indent}  <iID>{esc(iid)}</iID>', f'{indent}</moduleID>']
        return "\n".join(xml)
        
    def generate_property_list_xml(prop_list, indent="      "):
        if not prop_list: return ""
        xml = [f'{indent}<properties>']
        for prop in prop_list:
            if not prop['name'] or not prop['type'] or (prop['value'] is None): raise ValueError("Property: Name, Type, and Value are required.")
            if prop['immutable']:
                xml.append(f'{indent}  <Property name="{esc(prop["name"])}" type="{esc(prop["type"])}" unit="{esc(prop["unit"])}" description="{esc(prop["description"])}">')
                xml.append(f'{indent}    <value>{esc(prop["value"])}</value>')
                xml.append(f'{indent}    <immutable>true</immutable>')
                xml.append(f'{indent}  </Property>')
            else:
                xml.append(f'{indent}  <Property name="{esc(prop["name"])}" type="{esc(prop["type"])}" unit="{esc(prop["unit"])}" description="{esc(prop["description"])}" value="{esc(prop["value"])}" immutable="false" />')
        xml.append(f'{indent}</properties>')
        return "\n".join(xml)

    def generate_exe_form_xml(exe_form, tag_name="exeForm", indent="    "):
        if not exe_form['exeFileURL']: raise ValueError(f"ExeForm ({tag_name}): 'exeFileURL' is required.")
        xml = [f'{indent}<{tag_name}>', f'{indent}  <exeFileURL>{esc(exe_form["exeFileURL"])}</exeFileURL>']
        for cmd in exe_form['shellCmd']:
            if cmd['value']: xml.append(f'{indent}  <ShellCmd>{esc(cmd["value"])}</ShellCmd>')
        
        prop_xml_str = generate_property_list_xml(exe_form['properties'], indent + "  ")
        if prop_xml_str:
            # 属性名を <properties> から <Properties> に変更
            prop_xml_str = prop_xml_str.replace("<properties>", "<Properties>").replace("</properties>", "</Properties>")
            xml.append(prop_xml_str)
            
        if exe_form.get('use_addInfo', False) and exe_form.get('addInfo_nv'): xml.append(generate_nv_list_xml(exe_form['addInfo_nv'], indent + "  "))
            
        xml.append(f'{indent}</{tag_name}>')
        return "\n".join(xml)

    # --- XML生成ロジック本体 ---

    try:
        # --- 1. XML Lines Compilation ---
        xml_lines = []
        s_gen, s_id, s_prop, s_io, s_stat, s_serv, s_infra, s_safe, s_model, s_exec = st.session_state.gen_info, st.session_state.idn_type, st.session_state.properties, st.session_state.io_variables, st.session_state.status, st.session_state.services, st.session_state.infrastructure, st.session_state.safe_secure, st.session_state.modelling, st.session_state.executable_form
        
        # --- ModuleID String Extraction ---
        mid_string, iid_string = "", ""
        if s_id['id_input_mode'] == "Guided Generator":
            # (Guided ID generation logic)
            mid_vid = validate_hex(s_id['vid'], 32, "IDnType/VID")
            pid_type = 0
            if s_id['is_composite']:  pid_type |= (1 << 7)
            if s_id['pid_has_sw']:    pid_type |= (1 << 6)
            if s_id['pid_has_hw']:    pid_type |= (1 << 5)
            if s_id['pid_safety']:    pid_type |= (1 << 4)
            if s_id['pid_security']:  pid_type |= (1 << 3)
            mid_pid_type = f"{pid_type:02x}"
            mid_pid_vendor = validate_hex(s_id['pid_vendor'], 6, "IDnType/Vendor Specific PID")
            mid_pid = mid_pid_type + mid_pid_vendor
            mid_rev = validate_hex(s_id['rev'], 8, "IDnType/Revision")
            mid_serial = validate_hex(s_id['serialNo'], 8, "IDnType/Serial No")
            cat_l0 = ENUMS["L0_Category"][s_id['cat_l0']]
            cat_l1 = ENUMS["L1_Category"][s_id['cat_l1']]
            l1_key = s_id['cat_l1'].split(" (")[0]
            if l1_key not in ENUMS["L2_Category"]: l1_key = "Reserved"
            l2_map = ENUMS["L2_Category"][l1_key]
            # L2の選択肢が変更された場合のフォールバック
            if s_id['cat_l2'] not in l2_map:
                s_id['cat_l2'] = list(l2_map.keys())[0]
            cat_l2 = l2_map[s_id['cat_l2']]
            cat_l0_b = int(cat_l0, 2)
            cat_l1_b = int(cat_l1, 2)
            cat_l2_b = int(cat_l2, 2)
            cat_l3_hex_val = validate_hex(s_id['cat_l3_hex'], 2, "IDnType/Category L3")
            cat_l4_hex_val = validate_hex(s_id['cat_l4_hex'], 2, "IDnType/Category L4")
            cat_l3_b = int(cat_l3_hex_val, 16)
            cat_l4_b = int(cat_l4_hex_val, 16)
            cat_id_int = (cat_l0_b << 22) | (cat_l1_b << 18) | (cat_l2_b << 12) | (cat_l3_b << 6) | cat_l4_b
            mid_category = f"{cat_id_int:06x}"
            mid_string = mid_vid + mid_pid + mid_rev + mid_serial + mid_category
            iid_string = validate_hex(s_id['iid'], 2, "IDnType/Instance ID (IID)")
        elif s_id['id_input_mode'] == "Direct Input":
            mid_string = validate_hex(s_id['direct_mid'], 62, "IDnType/Direct mID")
            iid_string = validate_hex(s_id['direct_iid'], 2, "IDnType/Direct iID")

        # --- XML Lines Compilation ---
        
        xml_lines.append('<?xml version="1.0" encoding="UTF-8"?>')
        xml_lines.append('<Module type="SIM">')
        
        # 1. GenInfo
        xml_lines.append(f'  <moduleName>{esc(s_gen["moduleName"])}</moduleName>')
        if s_gen['description']: xml_lines.append(f'  <description>{esc(s_gen["description"])}</description>')
        xml_lines.append(f'  <manufacturer>{esc(s_gen["manufacturer"])}</manufacturer>')
        if s_gen['examples']: xml_lines.append(f'  <examples>{esc(s_gen["examples"])}</examples>')

        # 2. IDnType
        idn_type_attr = 'type="Com"' if s_id['is_composite'] else 'type="Bas"'
        xml_lines.append(f'  <idnType {idn_type_attr}><moduleID><mID>{esc(mid_string)}</mID><iID>{esc(iid_string)}</iID></moduleID>')
        xml_lines.append(f'    <informationModelVersion>{esc(s_id["imv"])}</informationModelVersion>')
        if s_id['is_composite'] and s_id['swAspects']:
            xml_lines.append(f'    <swAspects>')
            for aspect in s_id['swAspects']:
                xml_lines.append(f'      <moduleID><mID>{esc(validate_hex(aspect["mID"], 62, "SW Aspect mID"))}</mID><iID>{esc(validate_hex(aspect["iID"], 2, "SW Aspect iID"))}</iID></moduleID>')
            xml_lines.append(f'    </swAspects>')
        xml_lines.append(f'  </idnType>')

        # 3. Properties
        xml_lines.append(f'  <properties>')
        xml_lines.append(f'    <osType type="{esc(s_prop["os_type"])}" bit="{esc(s_prop["os_bit"])}" version="{esc(s_prop["os_version"])}" />')
        xml_lines.append(f'    <compiler><osName>{esc(s_prop["compiler_osName"])}</osName><verRangeOS min="{esc(s_prop["compiler_verRangeOS_min"])}" max="{esc(s_prop["compiler_verRangeOS_max"])}" /><compilerName>{esc(s_prop["compiler_name"])}</compilerName><verRangeCompiler min="{esc(s_prop["compiler_verRangeCompiler_min"])}" max="{esc(s_prop["compiler_verRangeCompiler_max"])}" /><bitnCPUarch>{esc(s_prop["compiler_bitnCPUarch"])}</bitnCPUarch></compiler>')
        xml_lines.append(f'    <exeType>')
        for exe in s_prop['exeTypes']:
            xml_lines.append(f'      <Item><opType>{esc(exe["opType"])}</opType><priority>{esc(int(exe["priority"]))}</priority><hardRT>{str(exe["hardRT"]).lower()}</hardRT><timeConstraint>{esc(exe["timeConstraint"])}</timeConstraint><instanceType>{esc(exe["instanceType"])}</instanceType></Item>')
        xml_lines.append(f'    </exeType>')
        if s_prop['use_libs'] and s_prop['libs']:
            xml_lines.append(f'    <Libraries>')
            for lib in s_prop['libs']:
                if lib['name'] or lib['version']: xml_lines.append(f'      <Item name="{esc(lib["name"])}" version="{esc(lib["version"])}" />')
            xml_lines.append(f'    </Libraries>')
        if s_id['is_composite']:
            xml_lines.append(f'    <organization>')
            xml_lines.append(f'      <owner>')
            xml_lines.append(f'        <mID>{esc(mid_string)}</mID>')
            xml_lines.append(f'        <iID>{esc(iid_string)}</iID>')
            xml_lines.append(f'      </owner>')
            xml_lines.append(f'      <dependency>{esc(s_prop["org_dependency"])}</dependency>')
            if not s_prop['org_members']: raise ValueError("Composite Module: At least one organization member is required.")
            for mem in s_prop['org_members']:
                xml_lines.append(f'      <member>')
                xml_lines.append(f'        <member>') # スキーマに基づく内側の <member> タグ
                xml_lines.append(f'          <mID>{esc(validate_hex(mem["mID"], 62, "Org Member mID"))}</mID>')
                xml_lines.append(f'          <iID>{esc(validate_hex(mem["iID"], 2, "Org Member iID"))}</iID>')
                xml_lines.append(f'        </member>')
                xml_lines.append(f'        <dependency>{esc(mem["dependency"])}</dependency>')
                xml_lines.append(f'      </member>')
            if s_prop['org_addInfo_use'] and s_prop['org_addInfo_nv']: xml_lines.append(generate_nv_list_xml(s_prop['org_addInfo_nv'], "      "))
            xml_lines.append(f'    </organization>')
        if s_prop['use_custom_props'] and s_prop['custom_props']:
            # <properties> タグを除去して <Property> タグのみを挿入
            prop_xml_str = generate_property_list_xml(s_prop['custom_props'], "    ")
            prop_xml_str = prop_xml_str.replace("    <properties>\n", "").replace("\n    </properties>", "")
            xml_lines.append(prop_xml_str)
        xml_lines.append(f'  </properties>')

        # 4. IOVariables
        xml_lines.append(f'  <ioVariables>')
        def generate_io_item_xml(item, tag_name):
            xml = []
            className = item.get("className", ""); className_attr = f'className="{esc(className)}"' if className else ""
            # 必須項目チェック
            if not item['name'] or not item['type'] or (item['value'] is None):
                 raise ValueError(f"IOVariables {tag_name}: Name, Type, and Value are required. Found: {item['name']}, {item['type']}, {item['value']}")
            
            if not item['use_addInfo'] and not item['use_moduleID'] and not className:
                xml.append(f'    <{tag_name} name="{esc(item["name"])}" type="{esc(item["type"])}" value="{esc(item["value"])}" unit="{esc(item["unit"])}" description="{esc(item["description"])}" />')
            else:
                xml.append(f'    <{tag_name} name="{esc(item["name"])}" type="{esc(item["type"])}" {className_attr}>')
                xml.append(f'      <value>{esc(item["value"])}</value>')
                if item["unit"]: xml.append(f'      <unit>{esc(item["unit"])}</unit>')
                if item["description"]: xml.append(f'      <description>{esc(item["description"])}</description>')
                if item.get('use_moduleID', False): xml.append(generate_module_id_xml(item, "      "))
                if item.get('use_addInfo', False) and item.get('addInfo_nv'): xml.append(generate_nv_list_xml(item['addInfo_nv'], "      "))
                xml.append(f'    </{tag_name}>')
            return "\n".join(xml)
        if s_io['inputs']:
            xml_lines.append('    <Inputs>')
            for item in s_io['inputs']: xml_lines.append(generate_io_item_xml(item, "Input"))
            xml_lines.append('    </Inputs>')
        if s_io['outputs']:
            xml_lines.append('    <Outputs>')
            for item in s_io['outputs']: xml_lines.append(generate_io_item_xml(item, "Output"))
            xml_lines.append('    </Outputs>')
        if s_io['inouts']:
            xml_lines.append('    <InOuts>')
            for item in s_io['inouts']: xml_lines.append(generate_io_item_xml(item, "Inout"))
            xml_lines.append('    </InOuts>')
        xml_lines.append(f'  </ioVariables>')

        # 5. Status
        xml_lines.append(f'  <Status>')
        xml_lines.append(f'    <executionStatus>{esc(s_stat["executionStatus"])}</executionStatus>')
        xml_lines.append(f'    <errorType>{esc(s_stat["errorType"])}</errorType>')
        xml_lines.append(f'  </Status>')

        # 6. Services
        xml_lines.append(f'  <services>')
        xml_lines.append(f'    <NoOfBasicService>{sum(1 for p in s_serv["profiles"] if p["moType"] == "MANDATORY")}</NoOfBasicService>')
        xml_lines.append(f'    <NoOfOptionalService>{sum(1 for p in s_serv["profiles"] if p["moType"] == "OPTIONAL")}</NoOfOptionalService>')
        for profile in s_serv['profiles']:
            if profile['methodList_type'] == 'IDL': 
                if not profile['ifURL']: raise ValueError("Service (IDL): ifURL is required.")
                xml_lines.append(f'    <ServiceIDL>{esc(profile["ifURL"])}</ServiceIDL>')
            elif profile['methodList_type'] == 'XML':
                if not profile['profile_id']: raise ValueError("Service (XML): Service ID is required.")
                xml_lines.append(f'    <ServiceXML>')
                xml_lines.append(f'      <ID>{esc(profile["profile_id"])}</ID>')
                xml_lines.append(f'      <pvType>{esc(profile["pvType"])}</pvType>')
                xml_lines.append(f'      <moType>{esc(profile["moType"])}</moType>')
                if profile.get('use_moduleID', False): xml_lines.append(generate_module_id_xml(profile, "      "))
                if profile['methodList']:
                    xml_lines.append(f'      <methodList>')
                    for method in profile['methodList']:
                        if not method['methodName']: raise ValueError(f"Service '{profile['profile_id']}': Method Name is required.")
                        xml_lines.append(f'        <Method>')
                        xml_lines.append(f'          <methodName>{esc(method["methodName"])}</methodName>')
                        for arg_idx, arg in enumerate(method['argType']):
                            if not arg['valueName'] or not arg['type']: raise ValueError(f"Service Method '{method['methodName']}': Argument Name and Type are required.")
                            arg_add_info_xml = generate_nv_list_xml(arg.get('addInfo_nv', []), "          ")
                            order_attr = f'order="{arg_idx + 1}"'
                            if arg_add_info_xml:
                                xml_lines.append(f'          <argType {order_attr}>')
                                xml_lines.append(f'            <type>{esc(arg["type"])}</type>')
                                xml_lines.append(f'            <valueName>{esc(arg["valueName"])}</valueName>')
                                xml_lines.append(f'            <inout>{esc(arg["inout"])}</inout>')
                                xml_lines.append(arg_add_info_xml) # generate_nv_list_xml が正しいインデントで出力
                                xml_lines.append(f'          </argType>')
                            else:
                                xml_lines.append(f'          <argType type="{esc(arg["type"])}" valueName="{esc(arg["valueName"])}" inout="{esc(arg["inout"])}" {order_attr} />')
                        xml_lines.append(f'          <retType>{esc(method["retType"])}</retType>')
                        xml_lines.append(f'          <moType>{esc(method["moType"])}</moType>')
                        xml_lines.append(f'          <reqProvType>{esc(method["reqProvType"])}</reqProvType>')
                        if method.get('use_addInfo', False) and method.get('addInfo_nv'): xml_lines.append(generate_nv_list_xml(method['addInfo_nv'], "          "))
                        xml_lines.append(f'        </Method>')
                    xml_lines.append(f'      </methodList>')
                if profile.get('use_addInfo', False) and profile.get('addInfo_nv'): xml_lines.append(generate_nv_list_xml(profile['addInfo_nv'], "      "))
                xml_lines.append(f'    </ServiceXML>')
        xml_lines.append(f'  </services>')

        # 7. Infrastructure
        xml_lines.append(f'  <infra>')
        def generate_infra_type_xml(item_list, tag_name, indent="    "):
            if not item_list: return
            xml_lines.append(f'{indent}<{tag_name}>')
            for item in item_list:
                if not item['name'] or not item['version_min'] or not item['version_max']: raise ValueError(f"Infrastructure {tag_name}: Name, Min Version, and Max Version are required.")
                xml_lines.append(f'{indent}  <InfraType>')
                xml_lines.append(f'{indent}    <Name>{esc(item["name"])}</Name>')
                xml_lines.append(f'{indent}    <Version min="{esc(item["version_min"])}" max="{esc(item["version_max"])}" />')
                xml_lines.append(f'{indent}  </InfraType>')
            xml_lines.append(f'{indent}</{tag_name}>')

        generate_infra_type_xml(s_infra['database'], "Database")
        generate_infra_type_xml(s_infra['middleware'], "Middleware")

        
        if s_infra['comms']:
            has_valid_comms = False
            comms_xml_lines = [] # 有効なcommsのXMLを一時的に保持

            for comm in s_infra['comms']:
                valid_protos = []
                for proto in comm['mostTopProtocol']:
                    # Name, Min, Max がすべて入力されている proto のみ「有効」とみなす
                    if proto['name'] and proto['version_min'] and proto['version_max']:
                        valid_protos.append(proto)
                    # どれか一つでも欠けているが、どれかは入力されている場合（＝入力途中）はエラー
                    elif proto['name'] or proto['version_min'] or proto['version_max']:
                        raise ValueError("Communication mostTopProtocol: Name, Min Version, and Max Version are all required if you start filling one.")



                if not valid_protos:
                    continue

                # この comm は有効。XMLを生成する
                has_valid_comms = True
                comms_xml_lines.append(f'      <Communication>')
                comms_xml_lines.append(f'        <mostTopProtocol>')
                for proto in valid_protos: # 有効な proto のみ出力
                    comms_xml_lines.append(f'          <InfraType>')
                    comms_xml_lines.append(f'            <Name>{esc(proto["name"])}</Name>')
                    comms_xml_lines.append(f'            <Version min="{esc(proto["version_min"])}" max="{esc(proto["version_max"])}" />')
                    comms_xml_lines.append(f'          </InfraType>')
                comms_xml_lines.append(f'        </mostTopProtocol>')

                # underlyingProtocol (DataBus) の処理 (元のコードを流用)
                dbus = comm['underlyingProtocol']
                if not dbus: raise ValueError("Communication: underlyingProtocol/Databus is required.")
                comms_xml_lines.append(f'        <underlyingProtocol>')
                comms_xml_lines.append(f'          <Databus>')
                comms_xml_lines.append(f'            <ConnectorType>{esc(dbus["connectionType"])}</ConnectorType>')
                comms_xml_lines.append(f'            <TypePhyMac>{esc(dbus["typePhyMac"])}</TypePhyMac>')
                if dbus['typeNetTrans']:
                    comms_xml_lines.append(f'            <TypeNetTrans>')
                    for net in dbus['typeNetTrans']:
                        if net['value']: comms_xml_lines.append(f'              <Item>{esc(net["value"])}</Item>')
                    comms_xml_lines.append(f'            </TypeNetTrans>')
                if dbus['typeApp']:
                    comms_xml_lines.append(f'            <TypeApp>')
                    for app in dbus['typeApp']:
                        if app['value']: comms_xml_lines.append(f'              <Item>{esc(app["value"])}</Item>')
                    comms_xml_lines.append(f'            </TypeApp>')
                    comms_xml_lines.append(f'            <Speed value="{esc(dbus["speed"])}" unit="kbps" />')
                if dbus.get('use_addInfo', False) and dbus.get('addInfo_nv'): comms_xml_lines.append(generate_nv_list_xml(dbus['addInfo_nv'], "            "))
                comms_xml_lines.append(f'          </Databus>')
                comms_xml_lines.append(f'        </underlyingProtocol>')
                comms_xml_lines.append(f'      </Communication>')

            # 有効な comms が1つでも見つかった場合のみ、<comms> タグと中身を出力
            if has_valid_comms:
                xml_lines.append(f'    <comms>')
                xml_lines.extend(comms_xml_lines)
                xml_lines.append(f'    </comms>')

        if s_infra.get('use_addInfo', False) and s_infra.get('addInfo_nv'): xml_lines.append(generate_nv_list_xml(s_infra['addInfo_nv'], "    "))
        xml_lines.append(f'  </infra>')

        # 8. SafeSecure
        s_safe = st.session_state.safe_secure
        xml_lines.append(f'  <safeSecure>')
        xml_lines.append(f'    <Safety>')
        if s_safe['overallValidSafetyLevelType'] in ['PL', 'BOTH']: xml_lines.append(f'      <overall mode="PL">{esc(s_safe["overallSafetyLevelPL"])}</overall>')
        if s_safe['overallValidSafetyLevelType'] in ['SIL', 'BOTH']: xml_lines.append(f'      <overall mode="SIL">{esc(s_safe["overallSafetyLevelSIL"])}</overall>')
        if s_safe['overallValidSafetyLevelType'] == 'NONE' and not s_safe['inSafetyLevel']:
             xml_lines.append(f'      <overall mode="PL">n</overall>') # 規格上、Safetyタグは必須だがoverallはO。ただし意味的に最低レベルを記述
        for sf in s_safe['inSafetyLevel']:
            pl_attr = f'eachSafetyLevelPL="{esc(sf["eachSafetyLevelPL"])}"' if sf['validSafetyLevelType'] in ['PL', 'BOTH'] else ''
            sil_attr = f'eachSafetyLevelSIL="{esc(sf["eachSafetyLevelSIL"])}"' if sf['validSafetyLevelType'] in ['SIL', 'BOTH'] else ''
            xml_lines.append(f'      <inSafetyLevel safetyFunctionType="{esc(sf["safetyFunctionType"])}" validSafetyLevelType="{esc(sf["validSafetyLevelType"])}" {pl_attr} {sil_attr} />')
        xml_lines.append(f'    </Safety>')
        xml_lines.append(f'    <Security>')
        xml_lines.append(f'      <CyberSecurity>')
        xml_lines.append(f'        <OverallCybSecurityLevel>{esc(s_safe["overallCybSecurityLevel"])}</OverallCybSecurityLevel>')
        for csf in s_safe['inCybSecurityLevel']: 
            xml_lines.append(f'        <inCybSecurityLevel type="{esc(csf["securityType"])}" value="{esc(csf["eachSecurityLevel"])}" />')
        xml_lines.append(f'      </CyberSecurity>')
        xml_lines.append(f'    </Security>')
        if s_safe.get('use_addInfo', False) and s_safe.get('addInfo_nv'): xml_lines.append(generate_nv_list_xml(s_safe['addInfo_nv'], "    "))
        xml_lines.append(f'  </safeSecure>')

        # 9. Modelling
        s_model = st.session_state.modelling
        xml_lines.append(f'  <modelling>')
        for sim in s_model['simulationModel']:
            if not sim['simulator']: raise ValueError("Modelling: Simulator Name is required.")
            xml_lines.append(f'    <simulationModel><simulator>{esc(sim["simulator"])}</simulator>')
            if not sim['mdf']: raise ValueError(f"Modelling '{sim['simulator']}': At least one 'mdf' group is required.")
            for mdf_group in sim['mdf']:
                if not mdf_group['type']: raise ValueError(f"Modelling '{sim['simulator']}': MDF Type is required.")
                xml_lines.append(f'      <mdf type="{esc(mdf_group["type"])}">')
                if not mdf_group['items']: raise ValueError(f"Modelling MDF '{mdf_group['type']}': At least one file path (Item) is required.")
                for item in mdf_group['items']:
                    if not item['value']: raise ValueError(f"Modelling MDF '{mdf_group['type']}': File path (Item) value is required.")
                    xml_lines.append(f'        <Item>{esc(item["value"])}</Item>')
                xml_lines.append(f'      </mdf>')
            for lib in sim['libraries']:
                if lib['value']: xml_lines.append(f'      <libraries>{esc(lib["value"])}</libraries>')
            for sw in sim['dynamicSW']: xml_lines.append(generate_exe_form_xml(sw, "dynamicSW", "      "))
            if sim.get('use_addInfo', False) and sim.get('addInfo_nv'): xml_lines.append(generate_nv_list_xml(sim['addInfo_nv'], "      "))
            xml_lines.append(f'    </simulationModel>')
        xml_lines.append(f'  </modelling>')

        # 10. ExecutableForm
        s_exec = st.session_state.executable_form
        xml_lines.append(f'  <execForm>')
        for lib_url in s_exec['LibraryURL']:
            if lib_url['value']: xml_lines.append(f'    <libraryURL>{esc(lib_url["value"])}</libraryURL>')
        
        if not s_exec['exeForm']: raise ValueError("ExecutableForm: At least one 'exeForm' item is required.")
        for exe in s_exec['exeForm']: xml_lines.append(generate_exe_form_xml(exe, "exeForm", "    "))
        xml_lines.append(f'  </execForm>')
        
        # --- End Module ---
        xml_lines.append('</Module>')
        
        return "\n".join(xml_lines)

    except Exception as e:
        st.error(f"XML Generation Error: {e}", icon="🔥")
        return f""

# --- 5. GUIタブの描画 ---

tab_list = [
    "📥 Import XML", # 変更: 新しいタブを追加
    "1. GenInfo", "2. IDnType", "3. Properties", "4. IOVariables", "5. Status", 
    "6. Services", "7. Infrastructure", "8. SafeSecure", "9. Modelling", "10. ExecutableForm",
    "👀 View XML"
]
tabs = st.tabs(tab_list)

tab_import = tabs[0] # 変更: 新しいタブの参照
tab_gen = tabs[1]    # 変更: インデックスずらし
tab_id = tabs[2]     # 変更: インデックスずらし
tab_props = tabs[3]  # 変更: インデックスずらし
tab_io = tabs[4]     # 変更: インデックスずらし
tab_status = tabs[5] # 変更: インデックスずらし
tab_serv = tabs[6]   # 変更: インデックスずらし
tab_infra = tabs[7]  # 変更: インデックスずらし
tab_safe_secure = tabs[8] # 変更: インデックスずらし
tab_modelling = tabs[9] # 変更: インデックスずらし
tab_exec_form = tabs[10] # 変更: インデックスずらし
tab_view = tabs[11]  # 変更: インデックスずらし


# -------------------------
# Tab 0: Import XML (NEW)
# -------------------------
with tab_import:
    st.header("📥 Import Existing XML Model")
    st.markdown("""
    Upload a pre-existing XML file compliant with ISO 22166-202. 
    The data will be parsed and used to populate all other tabs in this GUI.

    **Note:** This will **overwrite** any data currently entered in the GUI.
    """)
    
    uploaded_file = st.file_uploader(
        "Drag and drop your XML file here or click to browse",
        type=["xml"],
        accept_multiple_files=False,
        key="xml_uploader"
    )
    
    if uploaded_file is not None:
        st.info(f"File selected: `{uploaded_file.name}` ({uploaded_file.size} bytes)")
        
        # st.button の on_click を使用してコールバック関数を呼び出す
        st.button(
            "⬆️ Load Data into GUI", 
            on_click=parse_and_load_xml, 
            args=(uploaded_file,), 
            use_container_width=True, 
            type="primary"
        )

# -------------------------
# Tab 1: GenInfo
# -------------------------
with tab_gen:
    st.header("1. General Information (GenInfo)")
    st.markdown("Enter the basic identification information for the module.")
    s_gen = st.session_state.gen_info
    s_gen['moduleName'] = st.text_input("Module Name*", value=s_gen['moduleName'], help="Name of the module (M, 1)")
    s_gen['manufacturer'] = st.text_input("Manufacturer*", value=s_gen['manufacturer'], help="Name of the manufacturer (M, 1)")
    s_gen['description'] = st.text_input("Description (Optional)", value=s_gen['description'], help="Description of the module (O, 0..1)")
    s_gen['examples'] = st.text_area("Examples (Optional)", value=s_gen['examples'], help="Usage examples, etc. (O, 0..1)", height=100)
    
# -------------------------
# Tab 2: IDnType
# -------------------------
with tab_id:
    st.header("2. Identifier & Type (IDnType)")
    s_id = st.session_state.idn_type
    st.subheader("Module Type")
    s_id['is_composite'] = st.toggle("Is this a Composite Module?", value=s_id['is_composite'], help="OFF=Basic, ON=Composite. This makes Properties->Organization conditional (C).")
    st.divider()
    st.subheader("ModuleID (Compliant with Annex A)")
    s_id['id_input_mode'] = st.radio(
        "ModuleID Input Mode*", ("Guided Generator", "Direct Input"),
        index=("Guided Generator", "Direct Input").index(s_id['id_input_mode']), # 変更: indexを動的に設定
        captions=["Generate mID from components (Recommended)", "Input existing mID (31B) and iID (1B) directly"],
        horizontal=True, key="id_input_mode_radio" # 変更: keyを変更
    )
    
    if s_id['id_input_mode'] == "Guided Generator":
        st.markdown("Enter the components to build the `mID` (31 bytes) and `iID` (1 byte).")
        with st.container(border=True):
            st.markdown("##### 1. Vendor ID (VID) (16 Bytes)")
            s_id['vid'] = st.text_input("VID (32 hex chars)*", value=s_id['vid'], max_chars=32, help="Vendor ID based on IETF RFC 4122 (UUID v5) (M, 16B)")
        with st.container(border=True):
            st.markdown("##### 2. Product ID (PID) (4 Bytes)")
            c1, c2 = st.columns([1, 2])
            with c1:
                st.markdown("**Type (1 Byte)**")
                s_id['pid_has_sw'] = st.checkbox("SW Aspect", value=s_id['pid_has_sw'], help="Software Aspect (Bit 6)")
                s_id['pid_has_hw'] = st.checkbox("HW Aspect", value=s_id['pid_has_hw'], help="Hardware Aspect (Bit 5)")
                s_id['pid_safety'] = st.checkbox("Safety Mode", value=s_id['pid_safety'], help="Safety-related function (Bit 4)")
                s_id['pid_security'] = st.checkbox("Security Mode", value=s_id['pid_security'], help="Security-related function (Bit 3)")
                st.caption(f"Composite (Bit 7): {'ON' if s_id['is_composite'] else 'OFF'}")
            with c2:
                s_id['pid_vendor'] = st.text_input("Vendor PID (6 hex chars)*", value=s_id['pid_vendor'], max_chars=6, help="Vendor-specific product ID (M, 3B)")
        with st.container(border=True):
            st.markdown("##### 3. Revision (REV) (4 Bytes)") # ヘッダー追加
            s_id['rev'] = st.text_input("Revision (8 hex chars)*", value=s_id['rev'], max_chars=8, help="Revision number (M, 4B)")
        with st.container(border=True):
            st.markdown("##### 4. Serial Number (S/N) (4 Bytes)") # ヘッダー追加
            s_id['serialNo'] = st.text_input("Serial Number (8 hex chars)*", value=s_id['serialNo'], max_chars=8, help="Serial number. Set to '00000000' for SW modules (M, 4B)")
        with st.container(border=True):
            st.markdown("##### 5. Category ID (3 Bytes)")
            c1, c2, c3 = st.columns(3)
            s_id['cat_l0'] = c1.selectbox("Category L0 (Type)*", options=ENUMS["L0_Category"].keys(), index=list(ENUMS["L0_Category"].keys()).index(s_id['cat_l0']), help="Module type classification (Table A.2)")
            s_id['cat_l1'] = c2.selectbox("Category L1 (Function)*", options=ENUMS["L1_Category"].keys(), index=list(ENUMS["L1_Category"].keys()).index(s_id['cat_l1']), help="Module function classification (Table A.3)")
            l1_key = s_id['cat_l1'].split(" (")[0]
            if l1_key not in ENUMS["L2_Category"]: l1_key = "Reserved"
            l2_options = list(ENUMS["L2_Category"][l1_key].keys())
            current_l2_index = 0
            if s_id['cat_l2'] in l2_options:
                current_l2_index = l2_options.index(s_id['cat_l2'])
            else:
                s_id['cat_l2'] = l2_options[0] # L1変更時のフォールバック
            s_id['cat_l2'] = c3.selectbox("Category L2 (Sub-function)*", options=l2_options, index=current_l2_index, help="Module sub-function classification (Table A.3)")
            c1, c2 = st.columns(2)
            s_id['cat_l3_hex'] = c1.text_input("Category L3 (6 bits / 2 hex chars)*", value=s_id['cat_l3_hex'], max_chars=2, help="Vendor-defined classification (M, 6bit: 00-3F)")
            s_id['cat_l4_hex'] = c2.text_input("Category L4 (6 bits / 2 hex chars)*", value=s_id['cat_l4_hex'], max_chars=2, help="Vendor-defined classification (M, 6bit: 00-3F)")
        with st.container(border=True):
            st.markdown("##### 6. Instance ID (iID) (1 Byte)") # ヘッダー追加
            s_id['iid'] = st.text_input("iID (2 hex chars)*", value=s_id['iid'], max_chars=2, help="Instance ID (M, 1B). Default '00'")
    elif s_id['id_input_mode'] == "Direct Input":
        st.markdown("Input the `mID` (31 bytes) and `iID` (1 byte) values directly as hex strings.")
        with st.container(border=True):
            st.markdown("##### 1. Module ID (mID) (31 Bytes)") # ヘッダー追加
            s_id['direct_mid'] = st.text_input("mID (62 hex chars)*", value=s_id['direct_mid'], max_chars=62, help="Concatenated string of VID(16B)+PID(4B)+Rev(4B)+SerialNo(4B)+CatID(3B)")
        with st.container(border=True):
            st.markdown("##### 2. Instance ID (iID) (1 Byte)") # ヘッダー追加
            s_id['direct_iid'] = st.text_input("iID (2 hex chars)*", value=s_id['direct_iid'], max_chars=2, help="Instance ID (M, 1B).")
    st.divider()
    st.subheader("Information Model Version (imv)") # ヘッダー追加
    s_id['imv'] = st.text_input("Information Model Version*", value=s_id['imv'], help="The version of the standard this information model adheres to (M, 1)")
    st.divider()
    st.subheader("Software Aspects (swAspects)")
    st.markdown("If this is a Composite Module, list the component SW Module IDs (O, N)")
    if s_id['is_composite']:
        for i in range(len(s_id['swAspects'])):
            item = s_id['swAspects'][i]
            item_id = item['id']
            with st.container(border=True):
                st.markdown(f"**SW Aspect Member {i+1}**")
                c1, c2, c3 = st.columns([4, 2, 1])
                item['mID'] = c1.text_input("Member mID (62 hex chars)*", value=item['mID'], key=f"sw_mid_{item_id}", max_chars=62, help="Component module's mID (31B)")
                item['iID'] = c2.text_input("Member iID (2 hex chars)*", value=item['iID'], key=f"sw_iid_{item_id}", max_chars=2, help="Component module's iID (1B)")
                c3.button("Delete", on_click=del_sw_aspect, args=(item_id,), key=f"sw_del_{item_id}", use_container_width=True)
        st.button("➕ Add SW Aspect", on_click=add_sw_aspect)
    else:
        st.info("This section is enabled when 'Is this a Composite Module?' is ON.")

# -------------------------
# Tab 3: Properties
# -------------------------
with tab_props:
    st.header("3. Module Properties")
    s_prop = st.session_state.properties
    with st.expander("Operating System (osType)*", expanded=True):
        c1, c2, c3 = st.columns(3)
        # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
        os_bit_index = ENUMS["NoBit"].index(s_prop['os_bit']) if s_prop['os_bit'] in ENUMS["NoBit"] else 0
        s_prop['os_type'] = c1.text_input("OS Type*", value=s_prop['os_type'], help="OS name (e.g., Windows, Ubuntu) (M, 1)")
        s_prop['os_bit'] = c2.selectbox("OS Bit*", options=ENUMS["NoBit"], index=os_bit_index, help="OS bitness (M, 1)")
        s_prop['os_version'] = c3.text_input("OS Version*", value=s_prop['os_version'], help="OS version (M, 1)")
    with st.expander("Compiler (compiler)*", expanded=True):
        c1, c2 = st.columns(2)
        s_prop['compiler_osName'] = c1.text_input("Target OS Name*", value=s_prop['compiler_osName'], help="Compiler target OS name (M, 1)")
        s_prop['compiler_name'] = c2.text_input("Compiler/Interpreter Name*", value=s_prop['compiler_name'], help="Compiler name (e.g., GCC, MSVC) (M, 1)")
        c1, c2 = st.columns(2)
        with c1.container(border=True):
            st.markdown("**OS Version Range (RangeString)***")
            s_prop['compiler_verRangeOS_min'] = st.text_input("Min*", value=s_prop['compiler_verRangeOS_min'], key="compiler_os_min")
            s_prop['compiler_verRangeOS_max'] = st.text_input("Max*", value=s_prop['compiler_verRangeOS_max'], key="compiler_os_max")
        with c2.container(border=True):
            st.markdown("**Compiler Version Range (RangeString)***")
            s_prop['compiler_verRangeCompiler_min'] = st.text_input("Min*", value=s_prop['compiler_verRangeCompiler_min'], key="compiler_ver_min")
            s_prop['compiler_verRangeCompiler_max'] = st.text_input("Max*", value=s_prop['compiler_verRangeCompiler_max'], key="compiler_ver_max")
        s_prop['compiler_bitnCPUarch'] = st.text_input("Target Bit & CPU Arch*", value=s_prop['compiler_bitnCPUarch'], help="Target bit-size and CPU architecture (e.g., 64, X86) (M, 1)")
    with st.expander("Execution Types (exeType)*", expanded=True):
        st.markdown("Define execution types for module instances (M, N). At least one is required.")
        # (XMLインポート後、exeTypesが空になるケースに対応)
        if not s_prop['exeTypes']:
            add_exeType()
        
        for i in range(len(s_prop['exeTypes'])):
            exe = s_prop['exeTypes'][i]
            exe_id = exe['id']
            with st.container(border=True):
                st.markdown(f"**Execution Type {i+1}**")
                c1, c2, c3 = st.columns(3)
                # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
                op_type_index = ENUMS["OpTypes"].index(exe['opType']) if exe['opType'] in ENUMS["OpTypes"] else 0
                inst_type_index = ENUMS["InstanceType"].index(exe['instanceType']) if exe['instanceType'] in ENUMS["InstanceType"] else 0
                
                exe['opType'] = c1.selectbox("Operation Type (opType)*", options=ENUMS["OpTypes"], index=op_type_index, key=f"exe_op_{exe_id}")
                exe['instanceType'] = c2.selectbox("Instance Type (instanceType)*", options=ENUMS["InstanceType"], index=inst_type_index, key=f"exe_inst_{exe_id}")
                exe['hardRT'] = c3.checkbox("Hard Real-Time (hardRT)*", value=exe['hardRT'], key=f"exe_rt_{exe_id}")
                c1, c2, c3 = st.columns([2, 1, 1])
                exe['timeConstraint'] = c1.number_input("Time Constraint (μs)*", value=float(exe['timeConstraint']), min_value=0.0, format="%.1f", key=f"exe_time_{exe_id}", help="Period for PERIODIC, Deadline for EVENTDRIVEN (M, 1)")
                exe['priority'] = c2.number_input("Priority (0=high)*", value=int(exe['priority']), min_value=0, max_value=255, step=1, key=f"exe_pri_{exe_id}", help="Instance priority (M, 1)")
                c3.button("Delete", on_click=del_exeType, args=(exe_id,), key=f"exe_del_{exe_id}", use_container_width=True, disabled=len(s_prop['exeTypes']) <= 1)
        st.button("➕ Add Execution Type", on_click=add_exeType)
    with st.expander("Libraries (libs) (Optional)", expanded=False):
        s_prop['use_libs'] = st.checkbox("Define library information (Optional)", value=s_prop['use_libs'], key="use_libs")
        if s_prop['use_libs']:
            st.markdown("List of libraries used by the module (O, N)")
            for i in range(len(s_prop['libs'])):
                lib = s_prop['libs'][i]
                lib_id = lib['id']
                with st.container(border=True):
                    c1, c2, c3 = st.columns([2, 2, 1])
                    lib['name'] = c1.text_input("Library Name (Optional)", value=lib['name'], key=f"lib_name_{lib_id}")
                    lib['version'] = c2.text_input("Library Version (Optional)", value=lib['version'], key=f"lib_ver_{lib_id}")
                    c3.button("Delete", on_click=del_lib, args=(lib_id,), key=f"lib_del_{lib_id}", use_container_width=True)
            st.button("➕ Add Library", on_click=add_lib)
    with st.expander("Organization (organization) (Conditional)", expanded=False):
        st.markdown("This section is **Mandatory (C)** if 'Is this a Composite Module?' is ON in the `IDnType` tab.")
        if st.session_state.idn_type['is_composite']:
            st.success("Set as Composite Module. Organization info is required. (C, 1)")
            c1, c2 = st.columns(2)
            c1.text_input("Owner (ModuleID)*", value="This Module (ID from IDnType tab)", disabled=True, help="The owner is always this module itself (M, 1). The ID is set automatically during XML generation.")
            # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
            org_dep_index = ENUMS["DependencyType"].index(s_prop['org_dependency']) if s_prop['org_dependency'] in ENUMS["DependencyType"] else 0
            s_prop['org_dependency'] = c2.selectbox("Owner DependencyType*", options=ENUMS["DependencyType"], index=org_dep_index, key="org_dep", help="Dependency type of the owner (this module) (M, 1)")
            st.markdown("---")
            st.markdown("**Members (OrgMemberType)*** (M, N)")
            st.markdown("Component member modules of this composite module. At least one is required.")
            for i in range(len(s_prop['org_members'])):
                mem = s_prop['org_members'][i]
                mem_id = mem['id']
                with st.container(border=True):
                    st.markdown(f"**Member {i+1}**")
                    c1, c2 = st.columns(2)
                    mem['mID'] = c1.text_input("Member mID (62 hex chars)*", value=mem['mID'], key=f"mem_mid_{mem_id}", max_chars=62, help="Member's mID (M)")
                    mem['iID'] = c2.text_input("Member iID (2 hex chars)*", value=mem['iID'], key=f"mem_iid_{mem_id}", max_chars=2, help="Member's iID (M)")
                    c1, c2 = st.columns([2, 1])
                    # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
                    mem_dep_index = ENUMS["DependencyType"].index(mem['dependency']) if mem['dependency'] in ENUMS["DependencyType"] else 0
                    mem['dependency'] = c1.selectbox("Member DependencyType*", options=ENUMS["DependencyType"], index=mem_dep_index, key=f"mem_dep_{mem_id}", help="Dependency type of this member (M, 1)")
                    c2.button("Delete", on_click=del_org_member, args=(mem_id,), key=f"mem_del_{mem_id}", use_container_width=True)
            st.button("➕ Add Member", on_click=add_org_member)
            st.markdown("---")
            st.markdown("**Additional Info (NVList)** (O, 1 -> O, N)")
            render_additional_info_editor(s_prop, 'org', 'org_nv_counter')
        else:
            st.info("This section is enabled when 'Is this a Composite Module?' is ON in the `IDnType` tab.")
    with st.expander("Custom Properties (property) (Optional)", expanded=False):
        s_prop['use_custom_props'] = st.checkbox("Define custom properties (Optional)", value=s_prop['use_custom_props'], key="use_custom_props")
        if s_prop['use_custom_props']:
            st.markdown("Additional non-standard properties (O, N)")
            render_property_editor(s_prop['custom_props'], 'custom_prop_counter', 'props_custom')

# -------------------------
# Tab 4: IOVariables
# -------------------------
def render_io_variable_editor(var_list, var_type, counter_key):
    for i in range(len(var_list)):
        item = var_list[i]
        item_id = item['id']
        with st.container(border=True):
            st.markdown(f"**{var_type.capitalize()} Variable {i+1}**")
            c1, c2, c3 = st.columns([2, 1, 1])
            item['name'] = c1.text_input("Name*", value=item['name'], key=f"io_{var_type}_name_{item_id}")
            item['type'] = c2.text_input("Type (e.g., float32)*", value=item['type'], key=f"io_{var_type}_type_{item_id}")
            item['value'] = c3.text_input("Default Value*", value=item['value'], key=f"io_{var_type}_val_{item_id}", help="Value of the variable (M, 1)")
            c1, c2, c3 = st.columns([1, 2, 1])
            item['unit'] = c1.text_input("Unit (Optional)", value=item['unit'], key=f"io_{var_type}_unit_{item_id}")
            item['description'] = c2.text_input("Description (Optional)", value=item['description'], key=f"io_{var_type}_desc_{item_id}")
            item['className'] = c3.text_input("Class Name (Optional)", value=item['className'], key=f"io_{var_type}_class_{item_id}", help="Used for complex/nested types")
            render_module_id_input(item, f"io_{var_type}_{item_id}")
            render_additional_info_editor(item, f"io_{var_type}_{item_id}", counter_key)
            st.button("Delete", on_click=del_io_variable, args=(var_type, item_id), key=f"io_{var_type}_del_{item_id}", use_container_width=True)

with tab_io:
    st.header("4. Input/Output Variables (IOVariables)")
    st.markdown("Define the module's data interfaces (O, N). (Based on ISO 22166-201, Table 4.10-4.15 and 202, Table 19/20).")
    s_io = st.session_state.io_variables
    with st.expander("Inputs (O, N)"):
        render_io_variable_editor(s_io['inputs'], "inputs", 'io_nv_counter')
        st.button("➕ Add Input", on_click=add_io_variable, args=("inputs",))
    with st.expander("Outputs (O, N)"):
        render_io_variable_editor(s_io['outputs'], "outputs", 'io_nv_counter')
        st.button("➕ Add Output", on_click=add_io_variable, args=("outputs",))
    with st.expander("InOuts (O, N)"):
        render_io_variable_editor(s_io['inouts'], "inouts", 'io_nv_counter')
        st.button("➕ Add InOut", on_click=add_io_variable, args=("inouts",))

# -------------------------
# Tab 5: Status
# -------------------------
with tab_status:
    st.header("5. Module Status (Status)")
    st.markdown("Define the execution status and error reporting of the module (Table 21, 22).")
    s_stat = st.session_state.status
    # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
    exec_status_index = ENUMS["ExeStatus"].index(s_stat['executionStatus']) if s_stat['executionStatus'] in ENUMS["ExeStatus"] else 0
    s_stat['executionStatus'] = st.selectbox(
        "Execution Status*", options=ENUMS["ExeStatus"],
        index=exec_status_index,
        help="Execution status of the module instance (M, 1)"
    )
    s_stat['errorType'] = st.number_input(
        "Error Type*", value=int(s_stat['errorType']), step=1,
        help="Error code (e.g., POSIX.1003.1). 0 for no error. (M, 1)"
    )

# -------------------------
# Tab 6: Services
# -------------------------
with tab_serv:
    st.header("6. Module Services (Services)")
    st.markdown("Define the module's callable services/methods (O, N).")
    s_serv = st.session_state.services
    
    st.markdown(f"**Total Basic (Mandatory) Services:** `{sum(1 for p in s_serv['profiles'] if p['moType'] == 'MANDATORY')}`")
    st.markdown(f"**Total Optional Services:** `{sum(1 for p in s_serv['profiles'] if p['moType'] == 'OPTIONAL')}`")
    
    for i in range(len(s_serv['profiles'])):
        profile = s_serv['profiles'][i]
        p_id = profile['id']
        
        with st.container(border=True):
            st.subheader(f"Service Profile {i+1}: {profile.get('profile_id', 'New Profile')}")
            c1, c2, c3 = st.columns([2, 1, 1])
            profile['profile_id'] = c1.text_input("Service ID (Name)*", value=profile.get('profile_id', ''), key=f"sp_id_{p_id}")
            
            # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
            pv_type_index = ENUMS["PhysicalVirtual"].index(profile['pvType']) if profile['pvType'] in ENUMS["PhysicalVirtual"] else 0
            mo_type_index = ENUMS["MOType"].index(profile['moType']) if profile['moType'] in ENUMS["MOType"] else 0
            
            profile['pvType'] = c2.selectbox("Type (pvType)*", options=ENUMS["PhysicalVirtual"], index=pv_type_index, key=f"sp_pv_{p_id}")
            profile['moType'] = c3.selectbox("Type (moType)*", options=ENUMS["MOType"], index=mo_type_index, key=f"sp_mo_{p_id}")

            # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
            ml_type_index = ["XML", "IDL"].index(profile['methodList_type']) if profile['methodList_type'] in ["XML", "IDL"] else 0
            profile['methodList_type'] = st.radio(
                "Interface Definition Type*", ("XML", "IDL"),
                index=ml_type_index,
                key=f"sp_ml_type_{p_id}", horizontal=True,
                help="XML: Define methods below. IDL: Provide URL. (Only one can be provided)"
            )

            if profile['methodList_type'] == 'IDL':
                profile['ifURL'] = st.text_input("ifURL (IDL Path)*", value=profile.get('ifURL', ''), key=f"sp_url_{p_id}")
            
            elif profile['methodList_type'] == 'XML':
                st.markdown("**Method List (methodList)** (O, N)")
                for j in range(len(profile['methodList'])):
                    method = profile['methodList'][j]
                    m_id = method['id']
                    with st.container(border=True):
                        st.markdown(f"**Method {j+1}: {method.get('methodName', 'New Method')}**")
                        c1, c2 = st.columns(2)
                        method['methodName'] = c1.text_input("Method Name*", value=method['methodName'], key=f"sm_name_{p_id}_{m_id}")
                        method['retType'] = c2.text_input("Return Type (e.g., bool, void)*", value=method['retType'], key=f"sm_ret_{p_id}_{m_id}")
                        c1, c2 = st.columns(2)
                        
                        # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
                        m_mo_type_index = ENUMS["MOType"].index(method['moType']) if method['moType'] in ENUMS["MOType"] else 0
                        m_req_type_index = ENUMS["ReqProvType"].index(method['reqProvType']) if method['reqProvType'] in ENUMS["ReqProvType"] else 0
                        
                        method['moType'] = c1.selectbox("Method moType*", options=ENUMS["MOType"], index=m_mo_type_index, key=f"sm_mo_{p_id}_{m_id}")
                        method['reqProvType'] = c2.selectbox("Method reqProvType*", options=ENUMS["ReqProvType"], index=m_req_type_index, key=f"sm_req_{p_id}_{m_id}")
                        
                        st.markdown("**Arguments (argType)** (O, N)")
                        for k in range(len(method['argType'])):
                            arg = method['argType'][k]
                            a_id = arg['id']
                            with st.container(border=True):
                                c1, c2, c3, c4 = st.columns([1, 1, 1, 1])
                                arg['valueName'] = c1.text_input("Arg Name*", value=arg['valueName'], key=f"sa_name_{p_id}_{m_id}_{a_id}")
                                arg['type'] = c2.text_input("Arg Type*", value=arg['type'], key=f"sa_type_{p_id}_{m_id}_{a_id}")
                                
                                # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
                                io_type_index = ENUMS["InOutType"].index(arg['inout']) if arg['inout'] in ENUMS["InOutType"] else 0
                                arg['inout'] = c3.selectbox("Arg InOutType*", options=ENUMS["InOutType"], index=io_type_index, key=f"sa_io_{p_id}_{m_id}_{a_id}")
                                c4.button("Del Arg", on_click=del_service_arg, args=(method, a_id), key=f"sa_del_{p_id}_{m_id}_{a_id}", use_container_width=True)
                                render_additional_info_editor(arg, f"sa_{p_id}_{m_id}_{a_id}", 'service_nv_counter')
                        st.button("➕ Add Argument", on_click=add_service_arg, args=(method,), key=f"sm_add_arg_{p_id}_{m_id}", use_container_width=True)
                        st.markdown("---")
                        
                        render_additional_info_editor(method, f"sm_{p_id}_{m_id}", 'service_nv_counter')
                        st.button("Delete Method", on_click=del_service_method, args=(profile, m_id), key=f"sm_del_{p_id}_{m_id}", use_container_width=True)
                
                st.button("➕ Add Method", on_click=add_service_method, args=(profile,), key=f"sp_add_meth_{p_id}", use_container_width=True)

            render_module_id_input(profile, f"sp_{p_id}")
            render_additional_info_editor(profile, f"sp_{p_id}", 'service_nv_counter')
            st.button("Delete Service Profile", on_click=del_service_profile, args=(p_id,), key=f"sp_del_{p_id}", use_container_width=True)

    st.button("➕ Add Service Profile", on_click=add_service_profile)

# -------------------------
# Tab 7: Infrastructure
# -------------------------
with tab_infra:
    st.header("7. Module Infrastructure (Infrastructure)")
    s_infra = st.session_state.infrastructure
    
    def render_infra_type_editor(list_key, counter_key, title):
        for i in range(len(s_infra[list_key])):
            item = s_infra[list_key][i]
            item_id = item['id']
            with st.container(border=True):
                st.markdown(f"**{title} {i+1}**")
                c1, c2, c3, c4 = st.columns([2, 1, 1, 1])
                item['name'] = c1.text_input("Name*", value=item['name'], key=f"infra_{list_key}_name_{item_id}")
                item['version_min'] = c2.text_input("Min Version*", value=item['version_min'], key=f"infra_{list_key}_min_{item_id}")
                item['version_max'] = c3.text_input("Max Version*", value=item['version_max'], key=f"infra_{list_key}_max_{item_id}")
                c4.button("Delete", on_click=del_infra_item, args=(list_key, item_id), key=f"infra_{list_key}_del_{item_id}", use_container_width=True)

    with st.expander("Database (O, N)"):
        render_infra_type_editor("database", 'infra_db_counter', "Database")
        st.button("➕ Add Database", on_click=add_infra_item, args=("database", 'infra_db_counter'))

    with st.expander("Middleware (O, N)"):
        render_infra_type_editor("middleware", 'infra_mw_counter', "Middleware")
        st.button("➕ Add Middleware", on_click=add_infra_item, args=("middleware", 'infra_mw_counter'))

    with st.expander("Communications (comms) (O, N)"):
        st.markdown("Define communication protocols. (Optional, per ISO 4.2.7 if middleware is used)")
            
        for i in range(len(s_infra['comms'])):
            comm = s_infra['comms'][i]
            c_id = comm['id']
            with st.container(border=True):
                st.subheader(f"Communication Set {i+1}")
                
                st.markdown("**Most Top Protocol(s) (InfraType)*** (M, N)")
                # (XMLインポート後、mostTopProtocolが空になるケースに対応)
                if not comm['mostTopProtocol']:
                    add_infra_comm_proto(comm)
                
                for j in range(len(comm['mostTopProtocol'])):
                    proto = comm['mostTopProtocol'][j]
                    p_id = proto['id']
                    with st.container(border=True):
                        c1, c2, c3, c4 = st.columns([2, 1, 1, 1])
                        proto['name'] = c1.text_input("Protocol Name*", value=proto['name'], key=f"comm_{c_id}_proto_name_{p_id}")
                        proto['version_min'] = c2.text_input("Min Version*", value=proto['version_min'], key=f"comm_{c_id}_proto_min_{p_id}")
                        proto['version_max'] = c3.text_input("Max Version*", value=proto['version_max'], key=f"comm_{c_id}_proto_max_{p_id}")
                        c4.button("Del Proto", on_click=del_infra_comm_proto, args=(comm, p_id), key=f"comm_{c_id}_proto_del_{p_id}", use_container_width=True, disabled=len(comm['mostTopProtocol']) <= 1)
                st.button("➕ Add Top Protocol", on_click=add_infra_comm_proto, args=(comm,), key=f"comm_{c_id}_proto_add")

                st.markdown("**Underlying Protocol (DataBus)*** (M, 1)")
                # (XMLインポート後、underlyingProtocolが空になるケースに対応)
                if 'underlyingProtocol' not in comm or not comm['underlyingProtocol']:
                     comm['underlyingProtocol'] = {"connectionType": "RJ45", "typePhyMac": "Ethernet", "speed": 100000.0, "typeNetTrans": [], "typeApp": [], "use_addInfo": False, "addInfo_nv": []}
                
                dbus = comm['underlyingProtocol']
                with st.container(border=True):
                    c1, c2, c3 = st.columns(3)
                    dbus['connectionType'] = c1.text_input("Connection Type*", value=dbus['connectionType'], key=f"comm_{c_id}_db_conn")
                    dbus['typePhyMac'] = c2.text_input("Type Phy/Mac*", value=dbus['typePhyMac'], key=f"comm_{c_id}_db_phy")
                    dbus['speed'] = c3.number_input("Speed (Kbps)*", value=float(dbus['speed']), min_value=0.0, format="%.1f", key=f"comm_{c_id}_db_speed")
                    
                    c1, c2 = st.columns(2)
                    with c1:
                        st.markdown("**Network/Transport Protocols (O, N)**")
                        if 'typeNetTrans' not in dbus: dbus['typeNetTrans'] = [] # パース後用
                        for k in range(len(dbus['typeNetTrans'])):
                            net = dbus['typeNetTrans'][k]
                            n_id = net['id']
                            c1_1, c1_2 = st.columns([3, 1])
                            net['value'] = c1_1.text_input(f"Net/Trans Protocol {k+1}", value=net['value'], key=f"comm_{c_id}_db_net_val_{n_id}")
                            c1_2.button("Del", on_click=del_string_item, args=(dbus['typeNetTrans'], n_id), key=f"comm_{c_id}_db_net_del_{n_id}", use_container_width=True)
                        st.button("➕ Add Net/Trans", on_click=add_string_item, args=(dbus['typeNetTrans'], 'infra_comm_net_counter'), key=f"comm_{c_id}_db_net_add", use_container_width=True)
                    
                    with c2:
                        st.markdown("**Application Protocols (O, N)**")
                        if 'typeApp' not in dbus: dbus['typeApp'] = [] # パース後用
                        for k in range(len(dbus['typeApp'])):
                            app = dbus['typeApp'][k]
                            a_id = app['id']
                            c2_1, c2_2 = st.columns([3, 1])
                            app['value'] = c2_1.text_input(f"App Protocol {k+1}", value=app['value'], key=f"comm_{c_id}_db_app_val_{a_id}")
                            c2_2.button("Del", on_click=del_string_item, args=(dbus['typeApp'], a_id), key=f"comm_{c_id}_db_app_del_{a_id}", use_container_width=True)
                        st.button("➕ Add App", on_click=add_string_item, args=(dbus['typeApp'], 'infra_comm_app_counter'), key=f"comm_{c_id}_db_app_add", use_container_width=True)

                    render_additional_info_editor(dbus, f"comm_{c_id}_db", 'infra_comm_nv_counter')
                
                st.button("Delete Communication Set", on_click=del_infra_comm, args=(c_id,), key=f"comm_del_{c_id}", use_container_width=True)
        
        st.button("➕ Add Communication Set", on_click=add_infra_comm)

    with st.expander("Additional Info (Optional)"):
        render_additional_info_editor(s_infra, "infra_root", 'infra_nv_counter')

# -------------------------
# Tab 8: SafeSecure (NEW)
# -------------------------
with tab_safe_secure:
    st.header("8. Safety & Security (SafeSecure)")
    st.markdown("Define safety and security levels (Table 4.38).")
    s_safe = st.session_state.safe_secure
    
    with st.container(border=True):
        st.subheader("Overall Safety Levels* (M, 1)")
        # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
        plsil_type_index = ENUMS["PLSILType"].index(s_safe['overallValidSafetyLevelType']) if s_safe['overallValidSafetyLevelType'] in ENUMS["PLSILType"] else 0
        pl_level_index = ENUMS["SafetyLevelPL"].index(s_safe['overallSafetyLevelPL']) if s_safe['overallSafetyLevelPL'] in ENUMS["SafetyLevelPL"] else 0
        sil_level_index = ENUMS["SafetyLevelSIL"].index(s_safe['overallSafetyLevelSIL']) if s_safe['overallSafetyLevelSIL'] in ENUMS["SafetyLevelSIL"] else 0
        
        s_safe['overallValidSafetyLevelType'] = st.selectbox(
            "Overall Valid Safety Level Type*",
            options=ENUMS["PLSILType"],
            index=plsil_type_index,
            key="safe_overall_type",
            help="Representation mode of overall safety level (M, 1)"
        )
        c1, c2 = st.columns(2)
        s_safe['overallSafetyLevelPL'] = c1.selectbox(
            "Overall Safety Level (PL)*",
            options=ENUMS["SafetyLevelPL"],
            index=pl_level_index,
            key="safe_overall_pl",
            help="Set PL if type is PL or BOTH (M, 1)"
        )
        s_safe['overallSafetyLevelSIL'] = c2.selectbox(
            "Overall Safety Level (SIL)*",
            options=ENUMS["SafetyLevelSIL"],
            index=sil_level_index,
            key="safe_overall_sil",
            help="Set SIL if type is SIL or BOTH (M, 1)"
        )
        
    with st.container(border=True):
        st.subheader("Overall Security Levels* (M, 1)")
        # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
        phy_sec_index = ENUMS["SecurityLevelPhysical"].index(s_safe['overallPhySecurityLevel']) if s_safe['overallPhySecurityLevel'] in ENUMS["SecurityLevelPhysical"] else 0
        cyb_sec_index = ENUMS["SecurityLevelCyber"].index(s_safe['overallCybSecurityLevel']) if s_safe['overallCybSecurityLevel'] in ENUMS["SecurityLevelCyber"] else 0
        
        c1, c2 = st.columns(2)
        s_safe['overallPhySecurityLevel'] = c1.selectbox(
            "Overall Physical Security Level*",
            options=ENUMS["SecurityLevelPhysical"],
            index=phy_sec_index,
            key="safe_overall_phy_sec",
            help="Overall Physical Security of a module (M, 1)"
        )
        s_safe['overallCybSecurityLevel'] = c2.selectbox(
            "Overall Cyber Security Level*",
            options=ENUMS["SecurityLevelCyber"],
            index=cyb_sec_index,
            key="safe_overall_cyb_sec",
            help="Overall Cyber Security of a module (M, 1)"
        )
        
    with st.expander("Individual Safety Functions (inSafetyLevel) (O, N)"):
        st.markdown("Define individual safety functions and their levels (Table 4.34).")
        for i in range(len(s_safe['inSafetyLevel'])):
            item = s_safe['inSafetyLevel'][i]
            item_id = item['id']
            with st.container(border=True):
                st.markdown(f"**Safety Function {i+1}**")
                
                # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
                sf_type_index = ENUMS["SafetyType"].index(item['safetyFunctionType']) if item['safetyFunctionType'] in ENUMS["SafetyType"] else 0
                sf_valid_type_index = ENUMS["PLSILType"].index(item['validSafetyLevelType']) if item['validSafetyLevelType'] in ENUMS["PLSILType"] else 0
                sf_pl_index = ENUMS["SafetyLevelPL"].index(item['eachSafetyLevelPL']) if item['eachSafetyLevelPL'] in ENUMS["SafetyLevelPL"] else 0
                sf_sil_index = ENUMS["SafetyLevelSIL"].index(item['eachSafetyLevelSIL']) if item['eachSafetyLevelSIL'] in ENUMS["SafetyLevelSIL"] else 0
                
                c1, c2 = st.columns(2)
                item['safetyFunctionType'] = c1.selectbox(
                    "Safety Function Type*",
                    options=ENUMS["SafetyType"],
                    index=sf_type_index,
                    key=f"safe_fn_type_{item_id}"
                )
                item['validSafetyLevelType'] = c2.selectbox(
                    "Valid Safety Level Type*",
                    options=ENUMS["PLSILType"],
                    index=sf_valid_type_index,
                    key=f"safe_fn_valid_type_{item_id}"
                )
                c1, c2, c3 = st.columns([1, 1, 2])
                item['eachSafetyLevelPL'] = c1.selectbox(
                    "Safety Level (PL)*",
                    options=ENUMS["SafetyLevelPL"],
                    index=sf_pl_index,
                    key=f"safe_fn_pl_{item_id}"
                )
                item['eachSafetyLevelSIL'] = c2.selectbox(
                    "Safety Level (SIL)*",
                    options=ENUMS["SafetyLevelSIL"],
                    index=sf_sil_index,
                    key=f"safe_fn_sil_{item_id}"
                )
                c3.button("Delete Safety Function", on_click=del_safety_function, args=(item_id,), key=f"safe_fn_del_{item_id}", use_container_width=True)
        st.button("➕ Add Safety Function", on_click=add_safety_function)
        
    with st.expander("Individual Cyber Security Functions (inCybSecurityLevel) (O, N)"):
        st.markdown("Define individual cyber security functions and their levels (Table 4.37).")
        for i in range(len(s_safe['inCybSecurityLevel'])):
            item = s_safe['inCybSecurityLevel'][i]
            item_id = item['id']
            with st.container(border=True):
                st.markdown(f"**Cyber Security Function {i+1}**")
                
                # (indexの指定を、XMLインポート後に存在しないキーでエラーが出ないように修正)
                csf_type_index = ENUMS["SecurityType"].index(item['securityType']) if item['securityType'] in ENUMS["SecurityType"] else 0
                csf_level_index = ENUMS["SecurityLevelCyber"].index(item['eachSecurityLevel']) if item['eachSecurityLevel'] in ENUMS["SecurityLevelCyber"] else 0
                
                c1, c2, c3 = st.columns([2, 1, 1])
                item['securityType'] = c1.selectbox(
                    "Security Type*",
                    options=ENUMS["SecurityType"],
                    index=csf_type_index,
                    key=f"cyber_fn_type_{item_id}"
                )
                item['eachSecurityLevel'] = c2.selectbox(
                    "Security Level*",
                    options=ENUMS["SecurityLevelCyber"],
                    index=csf_level_index,
                    key=f"cyber_fn_level_{item_id}"
                )
                c3.button("Delete Cyber Security Function", on_click=del_cyber_security_function, args=(item_id,), key=f"cyber_fn_del_{item_id}", use_container_width=True)
        st.button("➕ Add Cyber Security Function", on_click=add_cyber_security_function)

    with st.expander("Additional Info (Optional)"):
        render_additional_info_editor(s_safe, "safe_root", 'safe_nv_counter')

# -------------------------
# Tab 9: Modelling (NEW)
# -------------------------
with tab_modelling:
    st.header("9. Modelling (Modelling)")
    st.markdown("Define simulation models for the module (O, N) (Table 31, 32).")
    s_model = st.session_state.modelling

    for i in range(len(s_model['simulationModel'])):
        sim = s_model['simulationModel'][i]
        sim_id = sim['id']
        with st.container(border=True):
            st.subheader(f"Simulation Model {i+1}")
            sim['simulator'] = st.text_input("Simulator Name*", value=sim['simulator'], key=f"model_sim_name_{sim_id}", help="e.g., Gazebo, Webots (M, 1)")

            # --- MDF (M, N) ---
            st.markdown("**Model Description Files (mdf)*** (M, N)")
            st.markdown("At least one MDF group (e.g., URDF, STL) is required.")
            # (XMLインポート後、mdfが空になるケースに対応)
            if not sim['mdf']:
                add_mdf_group(sim)

            for j in range(len(sim['mdf'])):
                mdf_group = sim['mdf'][j]
                mdf_id = mdf_group['id']
                with st.container(border=True):
                    c1, c2 = st.columns([1, 1])
                    mdf_group['type'] = c1.text_input("MDF Type (e.g., URDF)*", value=mdf_group['type'], key=f"model_mdf_type_{sim_id}_{mdf_id}")
                    c2.button("Delete MDF Group", on_click=del_mdf_group, args=(sim, mdf_id), key=f"model_mdf_del_group_{sim_id}_{mdf_id}", use_container_width=True, disabled=len(sim['mdf']) <= 1)

                    st.markdown("**File Paths (Items)*** (M, N)")
                    # (XMLインポート後、itemsが空になるケースに対応)
                    if not mdf_group['items']:
                        add_mdf_item(mdf_group)

                    for k in range(len(mdf_group['items'])):
                        item = mdf_group['items'][k]
                        item_id = item['id']
                        c1_1, c1_2 = st.columns([4, 1])
                        item['value'] = c1_1.text_input(f"File Path {k+1}*", value=item['value'], key=f"model_mdf_item_val_{sim_id}_{mdf_id}_{item_id}")
                        c1_2.button("Del Path", on_click=del_mdf_item, args=(mdf_group, item_id), key=f"model_mdf_item_del_{sim_id}_{mdf_id}_{item_id}", use_container_width=True, disabled=len(mdf_group['items']) <= 1)
                    st.button("➕ Add File Path", on_click=add_mdf_item, args=(mdf_group,), key=f"model_mdf_item_add_{sim_id}_{mdf_id}", use_container_width=True)
            st.button("➕ Add MDF Group (e.g., STL)", on_click=add_mdf_group, args=(sim,), key=f"model_mdf_group_add_{sim_id}")

            # --- Libraries (O, N, String) ---
            st.markdown("**Simulation Libraries (libraries)** (O, N)")
            for j in range(len(sim['libraries'])):
                lib = sim['libraries'][j]
                lib_id = lib['id']
                c1, c2 = st.columns([4, 1])
                lib['value'] = c1.text_input(f"Library Path {j+1}", value=lib['value'], key=f"model_lib_val_{sim_id}_{lib_id}")
                c2.button("Del Lib", on_click=del_string_item, args=(sim['libraries'], lib_id), key=f"model_lib_del_{sim_id}_{lib_id}", use_container_width=True)
            st.button("➕ Add Library Path", on_click=add_string_item, args=(sim['libraries'], 'model_lib_counter'), key=f"model_lib_add_{sim_id}", use_container_width=True)

            # --- DynamicSW (O, N, ExeForm) ---
            st.markdown("**Dynamic Software (dynamicSW)** (O, N)")
            render_exe_form_editor(sim['dynamicSW'], 'model_dyn_sw', f"model_{sim_id}_dynsw")

            # --- AdditionalInfo (O, 1) ---
            render_additional_info_editor(sim, f"model_{sim_id}", 'model_nv_counter')

            st.button("Delete Simulation Model", on_click=del_simulation_model, args=(sim_id,), key=f"model_sim_del_{sim_id}", use_container_width=True)

    st.button("➕ Add Simulation Model", on_click=add_simulation_model)

# -------------------------
# Tab 10: ExecutableForm (NEW)
# -------------------------
with tab_exec_form:
    st.header("10. Executable Form (ExecutableForm)")
    st.markdown("Define the executable files and libraries for the module (Table 33, 34).")
    s_exec = st.session_state.executable_form

    # --- LibraryURL (O, N, String) ---
    with st.expander("Library URLs (LibraryURL) (O, N)"):
        st.markdown("Binary or source code libraries used by the module.")
        for i in range(len(s_exec['LibraryURL'])):
            item = s_exec['LibraryURL'][i]
            item_id = item['id']
            c1, c2 = st.columns([4, 1])
            item['value'] = c1.text_input(f"Library URL {i+1}", value=item['value'], key=f"exec_lib_val_{item_id}")
            c2.button("Del URL", on_click=del_string_item, args=(s_exec['LibraryURL'], item_id), key=f"exec_lib_del_{item_id}", use_container_width=True)
        st.button("➕ Add Library URL", on_click=add_string_item, args=(s_exec['LibraryURL'], 'exec_lib_counter'), use_container_width=True)

    # --- exeForm (M, N, ExeForm) ---
    with st.expander("Executable Forms (exeForm)* (M, N)"):
        st.markdown("At least one executable form is mandatory.")
        # (XMLインポート後、exeFormが空になるケースに対応)
        if not s_exec['exeForm']:
            add_exe_form_item(s_exec['exeForm'], 'exec')

        render_exe_form_editor(s_exec['exeForm'], 'exec', "exec_form")

# -------------------------
# Tab 11: View XML
# -------------------------
with tab_view:
    st.header("👀 11. View & Export XML")

    # --- ファイル保存ロジック (方法 2: サーバー保存) ---
    st.subheader("Save to Server Folder (WSL/Ubuntu)")

    current_filename = st.session_state.filename_server
    if not current_filename:
        default_filename_str = st.session_state.gen_info['moduleName'].replace(' ', '_').replace('.', '')
        if not default_filename_str:
            default_filename_str = "SIM_Model_Default"
        current_filename = f"{default_filename_str}.xml"

    # ユーザーが編集できるファイル名
    st.session_state.filename_server = st.text_input(
        "File Name (e.g., model.xml)*",
        value=current_filename, # valueを更新されたcurrent_filenameに設定
        key="xml_filename_input" # keyを変更
    )
    # 保存パス
    st.session_state.save_path_server = st.text_input(
        "Destination Folder Path (WSL/Ubuntu)*",
        value=st.session_state.save_path_server,
        help=f"アプリが動作しているWSL/Ubuntu内のパスを指定してください。（例: {os.getcwd()}）"
    )

    # (on_save_to_server 関数は変更なし)
    def on_save_to_server():
        # 1. XMLを（再）生成・検証する
        xml_content = validate_and_generate_xml()
        st.session_state.xml_output = xml_content

        # 2. 生成に失敗したかチェック
        if not xml_content:
            st.error("XML generation failed. Please check other tabs for validation errors. File not saved.", icon="🚫")
            return

        # 3. ファイル名とパスを取得
        filename = st.session_state.filename_server
        file_path = st.session_state.save_path_server

        if not filename:
            st.error("File Name is required.", icon="🚫")
            return
        if not file_path:
            st.error("Destination Folder Path is required.", icon="🚫")
            return

        # 4. サーバーに保存
        success, message = save_xml_to_server(xml_content, file_path, filename)

        if success:
            st.success(f"File successfully saved to: {message}", icon="✅")
        else:
            st.error(f"Failed to save file: {message}", icon="🔥")

    # --- XML生成ボタン ---
    st.subheader("Generate & View XML")
    st.markdown("Click the 'Generate' button to validate all inputs and create the XML. The result will appear below.")

    col1, col2 = st.columns([1, 1])

    def on_generate_only():
        st.session_state.xml_output = validate_and_generate_xml()
        if not st.session_state.xml_output:
            st.toast("XML generation failed. Check tabs for errors.", icon="🚫")
        else:
            st.toast("XML Generated Successfully!", icon="🎉")

    col1.button("🔄 Generate/Update XML View", on_click=on_generate_only, use_container_width=True, type="primary")

    # サーバー保存ボタン（on_save_to_server を呼び出す）
    col2.button("💾 Save XML to Server Folder", on_click=on_save_to_server, use_container_width=True)

    st.divider()

    # --- XMLダウンロードロジック (方法 1: ブラウザダウンロード) ---
    st.subheader("Download XML (Browser)")
    st.download_button(
        label="⬇️ Download XML to Browser",
        data=st.session_state.xml_output,
        file_name=st.session_state.filename_server or "sim_model.xml", # サーバーのファイル名を共有
        mime="application/xml",
        use_container_width=True,
        disabled=not st.session_state.xml_output,
        help="Generates and downloads the XML file to your browser."
    )
    st.divider()

    # --- XML表示エリア ---
    st.subheader("XML Output Preview")
    if st.session_state.xml_output:
        st.code(st.session_state.xml_output, language='xml', line_numbers=True)
    else:
        st.info("Click 'Generate/Update XML View' or import an XML file to see the preview here.")
