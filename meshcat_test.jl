using MeshCat, GeometryTypes, CoordinateTransformations

vis = Visualizer()

setobject!(vis[:box1],
    HyperRectangle(Vec(0., 0, 0), Vec(0.1, 0.2, 0.3)))


anim = Animation()

atframe(anim, 0) do
    # within the context of atframe, calls to
    # `settransform!` and `setprop!` are intercepted
    # and recorded in `anim` instead of having any
    # effect on `vis`.
    settransform!(vis[:box1], Translation(0., 0, 0))
end

atframe(anim, 30) do
    settransform!(vis[:box1], Translation(0., 10, 0))
end

# `setanimation!()` actually sends the animation to the
# viewer. By default, the viewer will play the animation
# right away. To avoid that, you can also pass `play=false`.
setanimation!(vis, anim)
