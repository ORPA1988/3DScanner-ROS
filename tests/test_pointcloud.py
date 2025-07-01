from scanner.pointcloud import Point, save_ply


def test_save_ply(tmp_path):
    points = [Point(1.0, 2.0, 3.0)]
    out = tmp_path / "test.ply"
    save_ply(points, out)
    content = out.read_text().splitlines()
    assert content[0] == "ply"
    assert content[-1] == "1.0 2.0 3.0"
