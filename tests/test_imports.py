def test_core_packages_import():
    import data
    import demo
    import eval
    import maze
    import nav
    import sim

    assert data is not None
    assert demo is not None
    assert eval is not None
    assert maze is not None
    assert nav is not None
    assert sim is not None
