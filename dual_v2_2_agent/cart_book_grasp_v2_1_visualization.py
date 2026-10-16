"""V2.1 numbered head candidates; reuse original-RGB 3D book logger."""
def install_book_visualization(agent, pose_reader, parameter_reader):
    from cart_book_grasp_v2_visualization import install_book_visualization as install
    import rerun as rr
    import rerun.blueprint as rrb
    session=install(agent,pose_reader,parameter_reader)
    class LocalBookLogger(agent.RerunPipelineLogger):
        def send_default_blueprint(self):
            rr.send_blueprint(rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Vertical(
                        rrb.Spatial2DView(origin='/book/images/cart_filter', name='Numbered books: robot left to right'),
                        rrb.Tabs(
                            rrb.Spatial2DView(origin='/book/images/candidates', name='Head candidates / selection'),
                            rrb.Spatial2DView(origin='/book/images/candidate_review', name='SAM3 book / bookend filtering'),
                            rrb.Spatial2DView(origin='/book/images/wrist_rgb', name='Wrist context (no shared IDs)'),
                        ),
                        rrb.Spatial2DView(origin='/book/images/book', name='Selected book segmentation'),
                        rrb.Spatial2DView(origin='/book/images/spine', name='Spine + center'),
                        rrb.Spatial2DView(origin='/book/depth', name='Aligned depth (m)'),
                    ),
                    rrb.Vertical(
                        rrb.Spatial3DView(origin='/world', contents=['/world/book/**', '/world/environment/**'], name='Scene / book / targets / measured TCP'),
                        rrb.TextDocumentView(origin='/book/status', name='Spine center & depth'),
                        rrb.Tabs(
                            rrb.TextDocumentView(origin='/book/selection', name='Book numbers / selected target'),
                            rrb.TextDocumentView(origin='/book/motion', name='Trajectory legend'),
                            rrb.TextDocumentView(origin='/step5/execution/current_action', name='Current action'),
                            rrb.TextDocumentView(origin='/book/current_command', name='Actual waypoint'),
                            rrb.TextDocumentView(origin='/book/wrist', name='Spine tilt / J7'),
                        ),
                        row_shares=[.65, .2, .15],
                    ), column_shares=[.4, .6]),
                auto_layout=False, auto_views=False), make_active=True, make_default=True)

    agent.RerunPipelineLogger=LocalBookLogger
    return session
