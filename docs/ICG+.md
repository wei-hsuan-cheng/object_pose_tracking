# Extended Iterative Corresponding Geometry (ICG+)

## 1. Purpose and position in the tracking stack

**ICG+ (Extended Iterative Corresponding Geometry)** is a high-rate, model-based tracker for the six-degree-of-freedom pose of a known rigid object.
Given a metric mesh and an initial pose close to the true pose, it refines

$$
{}^{C}T_M =
\begin{bmatrix}
{}^{C}R_M & {}^{C}t_M\\
0 & 1
\end{bmatrix}
\in SE(3),
$$

where $M$ is the object-model frame and $C$ is the camera frame.

ICG+ combines three complementary sources of evidence:

| Modality | Evidence | Main benefit |
|---|---|---|
| Region | Foreground/background and surface-region colors | Aligns visible contours |
| Depth | Sparse 3D surface correspondences | Constrains metric translation and orientation |
| Texture | Local 2D features linked to 3D model points | Resolves geometric and color ambiguities |

The original ICG method fused region and depth information.
ICG+ adds local texture tracking and support for multiple surface regions.

The conceptual pipeline is

$$
\boxed{
\text{mesh + current pose + RGB(-D)}
\rightarrow
\text{multimodal correspondences}
\rightarrow
\text{one 6-DoF pose correction}
}
$$

ICG+ is a **local tracker**, not a global detector.
It normally initializes the current frame with the previous result:

$$
T_k^{(0)} = T_{k-1}^{+}.
$$

An external method such as FoundationPose can provide the initial pose or reinitialize ICG+ after tracking failure.

---

## 2. Pose-correction parameterization

Starting from the current pose estimate, ICG+ searches for a small correction

$$
\boldsymbol\theta =
\begin{bmatrix}
\boldsymbol\theta_r\\
\boldsymbol\theta_t
\end{bmatrix}
\in \mathbb R^6,
$$

where

- $\boldsymbol\theta_r\in\mathbb R^3$ is a rotation vector;
- $\boldsymbol\theta_t\in\mathbb R^3$ is a translation correction.

Define

$$
T(\boldsymbol\theta)=
\begin{bmatrix}
\exp([\boldsymbol\theta_r]_\times) & \boldsymbol\theta_t\\
0 & 1
\end{bmatrix},
$$

with

$$
[\mathbf a]_\times =
\begin{bmatrix}
0 & -a_z & a_y\\
a_z & 0 & -a_x\\
-a_y & a_x & 0
\end{bmatrix},
\qquad
[\mathbf a]_\times\mathbf b=\mathbf a\times\mathbf b.
$$

The correction is applied on the right:

$$
\boxed{
{}^{C}T_M^{+}
=
{}^{C}T_M\,T(\hat{\boldsymbol\theta})
}
$$

so the variation is expressed in the model frame.
For a model point ${}^{M}\mathbf X$, the first-order variation is

$$
{}^{M}\mathbf X(\boldsymbol\theta)
\approx
\left(I+[\boldsymbol\theta_r]_\times\right){}^{M}\mathbf X
+
\boldsymbol\theta_t.
$$

The paper's $T(\boldsymbol\theta)$ uses a rotation-vector exponential and a direct translation.
It is not exactly the full $SE(3)$ exponential of a twist, although both are equivalent to first order for small updates.

---

## 3. Multimodal probabilistic fusion

Let the observations be

$$
\mathcal D=
\{\mathcal D_d,\mathcal D_t,\mathcal D_r\}
$$

for depth, texture, and region evidence.
Assuming the modalities and cameras are conditionally independent,

$$
p(\boldsymbol\theta\mid\mathcal D)
=
\prod_{i=1}^{n_d}
p(\boldsymbol\theta\mid\mathcal D_{d_i})
\prod_{i=1}^{n_t}
p(\boldsymbol\theta\mid\mathcal D_{t_i})
\prod_{i=1}^{n_r}
p(\boldsymbol\theta\mid\mathcal D_{r_i}).
$$

Taking the logarithm converts the product into a sum:

$$
\ell(\boldsymbol\theta)
=
\log p(\boldsymbol\theta\mid\mathcal D)
=
\sum_i\ell_{d_i}
+
\sum_i\ell_{t_i}
+
\sum_i\ell_{r_i}.
$$

Therefore, the modality derivatives can be accumulated:

$$
\mathbf g
=
\sum_i\mathbf g_{d_i}
+
\sum_i\mathbf g_{t_i}
+
\sum_i\mathbf g_{r_i},
$$

$$
H
=
\sum_i H_{d_i}
+
\sum_i H_{t_i}
+
\sum_i H_{r_i}.
$$

ICG+ does **not** estimate three independent poses and average them.
All modalities contribute to the same six-dimensional optimization system.

Using the paper's log-likelihood maximization convention, the regularized Newton step is

$$
\boxed{
\hat{\boldsymbol\theta}
=
(-H+\Lambda)^{-1}\mathbf g
}
$$

with

$$
\Lambda=
\begin{bmatrix}
\lambda_r I_3 & 0\\
0 & \lambda_t I_3
\end{bmatrix}.
$$

Equivalently, defining the minimization cost $E=-\ell$,

$$
\boxed{
(H_E+\Lambda)\hat{\boldsymbol\theta}
=
-\mathbf g_E.
}
$$

The regularization limits large updates in weakly observed directions.
It is optimizer damping; it is not temporal filtering.

---

## 4. Depth modality

The depth modality resembles sparse point-to-plane ICP.

For a sampled model surface point $\mathbf X_j$, ICG+:

1. projects the point into the depth image;
2. searches a local neighborhood for an observed 3D point $\mathbf P_j$;
3. rejects the match if its distance is too large;
4. evaluates the displacement along the model surface normal $\mathbf N_j$.

A conceptual point-to-plane residual is

$$
r_{d,j}(\boldsymbol\theta)
=
\mathbf N_j^\top
\left(
\mathbf X_j(\boldsymbol\theta)-\mathbf P_j
\right).
$$

The likelihood is modeled as Gaussian:

$$
p_{d,j}(\boldsymbol\theta)
\propto
\exp\left[
-\frac{r_{d,j}(\boldsymbol\theta)^2}
{2(d_{Z,j}\sigma_d)^2}
\right],
$$

where $\sigma_d$ is the depth uncertainty parameter and $d_{Z,j}$ is the observed depth.
The $d_Z$ factor accounts for the fact that depth uncertainty and measurement sparsity normally increase with distance.

Let

$$
J_{d,j}
=
\frac{\partial r_{d,j}}{\partial\boldsymbol\theta}.
$$

For the corresponding weighted least-squares cost,

$$
\mathbf g_d
=
\sum_j
\frac{1}{(d_{Z,j}\sigma_d)^2}
J_{d,j}^{\top}r_{d,j},
$$

$$
H_d
\approx
\sum_j
\frac{1}{(d_{Z,j}\sigma_d)^2}
J_{d,j}^{\top}J_{d,j}.
$$

Depth strongly constrains motion normal to a visible surface.
A large planar surface may still provide little information about translation within the plane or rotation about its normal; region and texture evidence complement these weak directions.

---

## 5. Region modality

The region modality aligns projected model contours with RGB observations.
Before tracking, ICG+ renders the mesh from many viewpoints and stores sparse geometric information, including:

- 3D contour and surface points;
- surface and contour-normal directions;
- valid foreground and background search distances.

At runtime, it selects the precomputed viewpoint closest to the current orientation.
For each projected contour point, it constructs a short correspondence line

$$
\mathbf x(r)=\mathbf c+r\mathbf n,
$$

where $\mathbf c$ is the projected contour point, $\mathbf n$ is its projected outward normal, and $r$ parameterizes position along the line.

Color histograms model the probability that pixel color $\mathbf y$ belongs to the foreground or background:

$$
p(\mathbf y\mid m_f),
\qquad
p(\mathbf y\mid m_b).
$$

For a hypothesized contour displacement $d$, the line likelihood has the conceptual form

$$
p_l(d)
\propto
\prod_{r\in\omega_l}
\left[
h_f(r-d)p_f(r)+h_b(r-d)p_b(r)
\right],
$$

where $h_f$ and $h_b$ are smoothed step functions describing the expected foreground/background transition.

Because the contour displacement depends on the object pose,

$$
d_l=d_l(\boldsymbol\theta),
$$

the pose derivative follows from the chain rule:

$$
\frac{\partial\log p_l}{\partial\boldsymbol\theta}
=
\frac{\partial\log p_l}{\partial d_l}
\frac{\partial d_l}{\partial\mathbf X_l}
\frac{\partial\mathbf X_l}{\partial\boldsymbol\theta}.
$$

This differs from ordinary edge detection.
The tracker does not merely choose the strongest intensity gradient; it seeks the location that best separates the learned foreground and background color distributions.

### Multi-region extension

Original ICG treats the object as one foreground region.
ICG+ can define several surface regions:

$$
\mathcal R_1,\mathcal R_2,\ldots,\mathcal R_m.
$$

Each region contributes its own evidence:

$$
p_r(\boldsymbol\theta)
=
\prod_{j=1}^{m}
p(\boldsymbol\theta\mid\mathcal R_j).
$$

This permits internal color boundaries, not only the outer silhouette, to constrain pose.
The regions are supplied as separate surface meshes.

---

## 6. Texture modality

The texture modality uses local image features such as ORB or SIFT.
It does not require a textured CAD model.
Instead, the tracker creates keyframes online:

1. detect image features on the tracked object;
2. render the model depth at the estimated pose;
3. back-project each valid feature onto the model surface;
4. store its descriptor and corresponding 3D model point.

A stored feature is

$$
\left({}^{M}\mathbf X_i,\mathbf d_i\right).
$$

If the descriptor matches a current image point $\mathbf x_i'$, the predicted projection is

$$
\mathbf x_i(\boldsymbol\theta)
=
\pi\left(
{}^{C}T_M
T(\boldsymbol\theta)
{}^{M}\widetilde{\mathbf X}_i
\right),
$$

where $\pi(\cdot)$ is the camera projection.
The reprojection residual is

$$
\mathbf r_{t,i}(\boldsymbol\theta)
=
\mathbf x_i'
-
\mathbf x_i(\boldsymbol\theta).
$$

A robust texture cost is

$$
E_t(\boldsymbol\theta)
=
\sum_i
\frac{1}{2\sigma_t^2}
\rho_{\mathrm{Tukey}}
\left(
\left\|\mathbf r_{t,i}(\boldsymbol\theta)\right\|^2
\right).
$$

For scalar residual magnitude $x$, the Tukey biweight is

$$
\rho_{\mathrm{Tukey}}(x)=
\begin{cases}
\dfrac{c^2}{6}
\left[
1-
\left(
1-\left(\dfrac{x}{c}\right)^2
\right)^3
\right],
& |x|\le c,\\[8pt]
\dfrac{c^2}{6},
& |x|>c.
\end{cases}
$$

The bounded loss limits the influence of incorrect descriptor matches.
Using iteratively reweighted least squares,

$$
E_t
\approx
\sum_i
\frac{w_i}{2\sigma_t^2}
\left\|\mathbf r_{t,i}\right\|^2.
$$

For a camera-frame point ${}^{C}\mathbf X_i=[X_i,Y_i,Z_i]^\top$, the pinhole projection Jacobian is

$$
\frac{\partial\mathbf x_i}{\partial{}^{C}\mathbf X_i}
=
\begin{bmatrix}
\dfrac{f_x}{Z_i} & 0 & -\dfrac{f_xX_i}{Z_i^2}\\[6pt]
0 & \dfrac{f_y}{Z_i} & -\dfrac{f_yY_i}{Z_i^2}
\end{bmatrix}.
$$

For the model-frame pose correction,

$$
\frac{\partial{}^{C}\mathbf X_i}
{\partial\boldsymbol\theta}
=
{}^{C}R_M
\begin{bmatrix}
-[{}^{M}\mathbf X_i]_\times & I_3
\end{bmatrix}.
$$

Therefore,

$$
J_{t,i}
=
\frac{\partial\mathbf x_i}{\partial{}^{C}\mathbf X_i}
\frac{\partial{}^{C}\mathbf X_i}{\partial\boldsymbol\theta}.
$$

The Gauss-Newton contributions are

$$
\mathbf g_t
\approx
\sum_i
\frac{w_i}{\sigma_t^2}
J_{t,i}^{\top}
\left(\mathbf x_i-\mathbf x_i'\right),
$$

$$
H_t
\approx
\sum_i
\frac{w_i}{\sigma_t^2}
J_{t,i}^{\top}J_{t,i}.
$$

Texture is especially useful when surface geometry is ambiguous—for example, rotation around the axis of a cylinder.

---

## 7. Complete per-frame iteration

For every camera frame:

1. initialize with the previous pose;
2. render/select the current model representation;
3. generate region, depth, and texture correspondences;
4. compute the gradient and Hessian contributions;
5. assemble and solve the regularized $6\times6$ Newton system;
6. update the pose;
7. recompute correspondences and repeat.

In equations,

$$
\begin{aligned}
\mathbf g &=
\mathbf g_r+\mathbf g_d+\mathbf g_t,\\
H &= H_r+H_d+H_t,\\
(-H+\Lambda)\hat{\boldsymbol\theta}
&=\mathbf g,\\
T &\leftarrow T\,T(\hat{\boldsymbol\theta}).
\end{aligned}
$$

Earlier refinement iterations use larger correspondence search ranges.
Later iterations reduce the search range for more precise alignment.

---

## 8. Why ICG+ is computationally efficient

ICG+ avoids dense full-image optimization:

- contours are represented by sparse one-dimensional correspondence lines;
- depth uses sparse surface samples;
- texture features are detected in a restricted image region;
- viewpoint-dependent model data are precomputed;
- each pose iteration solves only a $6\times6$ system.

The paper reports $312.4\ \mathrm{Hz}$ for its ORB configuration on one core of an Intel i9-11900K desktop CPU.
This number should not be transferred directly to another platform such as Jetson Orin, but it demonstrates the method's high-rate design.

---

## 9. Practical interpretation and limitations

$$
\boxed{
\text{ICG+ is an iterative multimodal pose-measurement optimizer.}
}
$$

It does not itself provide:

- global pose initialization;
- velocity or acceleration estimation;
- temporal Kalman filtering;
- future-motion prediction;
- guaranteed recovery after tracking loss.

It can fail when:

- the initial pose or inter-frame motion is too large;
- the object is fully or persistently occluded;
- the mesh or camera calibration is inaccurate;
- foreground and background have similar colors;
- depth is unreliable on transparent, reflective, or thin geometry;
- the object has unresolved geometric or visual symmetry.

For a robot-mounted camera, robot forward kinematics can first compensate for camera ego-motion:

$$
{}^{C_k}\hat T_{O,k}^{-}
=
\left({}^{B}T_C(q_k)\right)^{-1}
{}^{B}\hat T_{O,k}^{-},
$$

after which ICG+ refines the predicted camera-frame pose.

---

## 10. Relationship to M3T

For one rigid body, ICG+ produces a local visual gradient and Hessian:

$$
\boxed{
(\mathbf g_i,H_i)\in\mathbb R^6\times\mathbb R^{6\times6}.
}
$$

M3T reuses this kind of rigid-body evidence for multiple connected bodies.
It projects the per-body derivatives into joint coordinates and enforces kinematic-loop constraints.
Therefore, the clean conceptual order is

$$
\boxed{
\text{ICG}
\rightarrow
\text{ICG+}
\rightarrow
\text{M3T multi-body extension}.
}
$$

## References

- [M. Stoiber et al., ICG+: A Multi-Modal Approach for 6DoF Object Tracking, 2023](https://arxiv.org/pdf/2302.11458)
- [M. Stoiber et al., Iterative Corresponding Geometry: Fusing Region and Depth for Highly Efficient 3D Tracking, CVPR 2022](https://elib.dlr.de/189883/1/Stoiber_Iterative_Corresponding_Geometry_Fusing_Region_and_Depth_for_Highly_Efficient_CVPR_2022_paper.pdf)
- [Official implementation of ICG+](https://github.com/DLR-RM/3DObjectTracking/tree/master/ICG%2B)
