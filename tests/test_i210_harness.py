from pathlib import Path
from scripts.run_i210_baseline import find_tripinfo


def test_tripinfo_found_after_upstream_moves_output(tmp_path):
    directory=tmp_path/'Output_randomseed_5'; directory.mkdir()
    file=directory/'trace_tripinfo_5_25200.xml'; file.write_text('<tripinfos/>')
    assert find_tripinfo(tmp_path,5,25200)==file
