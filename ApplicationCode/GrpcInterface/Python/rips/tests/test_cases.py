import sys
import os
import math
import pytest 
import grpc
import tempfile

sys.path.insert(1, os.path.join(sys.path[0], '../../'))
import rips

import dataroot

def test_Launch(rips_instance, initialize_test):
    assert(rips_instance is not None)

def test_EmptyProject(rips_instance, initialize_test):
    cases = rips_instance.project.cases()
    assert(len(cases) is 0)

def test_OneCase(rips_instance, initialize_test):
    case = rips_instance.project.load_case(dataroot.PATH + "/TEST10K_FLT_LGR_NNC/TEST10K_FLT_LGR_NNC.EGRID")
    assert(case.name == "TEST10K_FLT_LGR_NNC")
    assert(case.case_id == 0)
    cases = rips_instance.project.cases()
    assert(len(cases) is 1)

def test_BoundingBox(rips_instance, initialize_test):
    case = rips_instance.project.load_case(dataroot.PATH + "/TEST10K_FLT_LGR_NNC/TEST10K_FLT_LGR_NNC.EGRID")
    assert(case.name == "TEST10K_FLT_LGR_NNC")
    boundingbox = case.reservoir_boundingbox()
    assert(math.isclose(3382.90, boundingbox.min_x, abs_tol=1.0e-1))
    assert(math.isclose(5850.48, boundingbox.max_x, abs_tol=1.0e-1))
    assert(math.isclose(4157.45, boundingbox.min_y, abs_tol=1.0e-1))
    assert(math.isclose(7354.93, boundingbox.max_y, abs_tol=1.0e-1))
    assert(math.isclose(-4252.61, boundingbox.min_z, abs_tol=1.0e-1))
    assert(math.isclose(-4103.60, boundingbox.max_z, abs_tol=1.0e-1))

    min_depth, max_depth = case.reservoir_depth_range()
    assert(math.isclose(4103.60, min_depth, abs_tol=1.0e-1))
    assert(math.isclose(4252.61, max_depth, abs_tol=1.0e-1))

def test_MultipleCases(rips_instance, initialize_test):
    case_paths = []
    case_paths.append(dataroot.PATH + "/TEST10K_FLT_LGR_NNC/TEST10K_FLT_LGR_NNC.EGRID")
    case_paths.append(dataroot.PATH + "/TEST10K_FLT_LGR_NNC/TEST10K_FLT_LGR_NNC.EGRID")
    case_paths.append(dataroot.PATH + "/TEST10K_FLT_LGR_NNC/TEST10K_FLT_LGR_NNC.EGRID")
    
    case_names = []
    for case_path in case_paths:
        case_name = os.path.splitext(os.path.basename(case_path))[0]
        case_names.append(case_name)
        rips_instance.project.load_case(path=case_path)

    cases = rips_instance.project.cases()
    assert(len(cases) == len(case_names))
    for i, case_name in enumerate(case_names):
        assert(case_name == cases[i].name)

def get_cell_index_with_ijk(cell_info, i, j, k):
    for (idx, cell) in enumerate(cell_info):
        if cell.local_ijk.i == i and cell.local_ijk.j == j and cell.local_ijk.k == k:
            return idx
    return -1

def test_10k(rips_instance, initialize_test):
    case_path = dataroot.PATH + "/TEST10K_FLT_LGR_NNC/TEST10K_FLT_LGR_NNC.EGRID"
    case = rips_instance.project.load_case(path=case_path)
    assert(len(case.grids()) == 2)
    cell_count_info = case.cell_count()
    assert(cell_count_info.active_cell_count == 11125)
    assert(cell_count_info.reservoir_cell_count == 316224)
    time_steps = case.time_steps()
    assert(len(time_steps) == 9)
    days_since_start = case.days_since_start()
    assert(len(days_since_start) == 9)
    cell_info = case.cell_info_for_active_cells()
    assert(len(cell_info) == cell_count_info.active_cell_count)

    # Check an active cell (found in resinsight ui)
    cell_index = get_cell_index_with_ijk(cell_info, 23, 44, 19)
    assert(cell_index != -1)

    cell_centers = case.active_cell_centers()
    assert(len(cell_centers) == cell_count_info.active_cell_count)

    # Check the cell center for the specific cell
    assert(math.isclose(3627.0, cell_centers[cell_index].x, abs_tol=0.5))
    assert(math.isclose(5210.0, cell_centers[cell_index].y, abs_tol=0.5))
    assert(math.isclose(4179.6, cell_centers[cell_index].z, abs_tol=0.5))

def test_PdmObject(rips_instance, initialize_test):
    case_path = dataroot.PATH + "/TEST10K_FLT_LGR_NNC/TEST10K_FLT_LGR_NNC.EGRID"
    case = rips_instance.project.load_case(path=case_path)
    assert(case.case_id == 0)
    assert(case.address() is not 0)
    assert(case.class_keyword() == "EclipseCase")
    case_id = case.get_value('CaseId')
    assert(case_id == case.case_id)

@pytest.mark.skipif(sys.platform.startswith('linux'), reason="Brugge is currently exceptionally slow on Linux")
def test_brugge_0010(rips_instance, initialize_test):
    case_path = dataroot.PATH + "/Case_with_10_timesteps/Real10/BRUGGE_0010.EGRID"
    case = rips_instance.project.load_case(path=case_path)
    assert(len(case.grids()) == 1)
    cellCountInfo = case.cell_count()
    assert(cellCountInfo.active_cell_count == 43374)
    assert(cellCountInfo.reservoir_cell_count == 60048)
    time_steps = case.time_steps()
    assert(len(time_steps) == 11)
    days_since_start = case.days_since_start()
    assert(len(days_since_start) == 11)

@pytest.mark.skipif(sys.platform.startswith('linux'), reason="Brugge is currently exceptionally slow on Linux")
def test_replaceCase(rips_instance, initialize_test):
    project = rips_instance.project.open(dataroot.PATH + "/TEST10K_FLT_LGR_NNC/10KWithWellLog.rsp")
    case_path = dataroot.PATH + "/Case_with_10_timesteps/Real0/BRUGGE_0000.EGRID"
    case = project.case(case_id=0)
    assert(case is not None)
    assert(case.name == "TEST10K_FLT_LGR_NNC")
    assert(case.case_id == 0)
    cases = rips_instance.project.cases()
    assert(len(cases) is 1)

    case.replace(new_grid_file=case_path)
    # Check that the case object has been changed
    assert(case.name == "Real0--BRUGGE_0000.EGRID")
    assert(case.case_id == 0)

    cases = rips_instance.project.cases()
    assert(len(cases) is 1)
    # Check that retrieving the case object again will yield the changed object
    case = project.case(case_id=0)
    assert(case.name == "Real0--BRUGGE_0000.EGRID")
    assert(case.case_id == 0)
    
def test_loadNonExistingCase(rips_instance, initialize_test):
    case_path = "Nonsense/Nonsense/Nonsense"
    with pytest.raises(grpc.RpcError):
        assert rips_instance.project.load_case(case_path)

@pytest.mark.skipif(sys.platform.startswith('linux'), reason="Brugge is currently exceptionally slow on Linux")
def test_exportFlowCharacteristics(rips_instance, initialize_test):
    case_path = dataroot.PATH + "/Case_with_10_timesteps/Real0/BRUGGE_0000.EGRID"
    case = rips_instance.project.load_case(case_path)
    with tempfile.TemporaryDirectory(prefix="rips") as tmpdirname:
        print("Temporary folder: ", tmpdirname)
        file_name = tmpdirname + "/exportFlowChar.txt"
        case.export_flow_characteristics(time_steps=8, producers=[], injectors = "I01", file_name = file_name)

