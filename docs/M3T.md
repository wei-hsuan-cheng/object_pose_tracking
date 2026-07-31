# M3T: Multi-Body Tracking

## 1. What M3T adds to rigid-object tracking

M3T extends a local six-degree-of-freedom rigid-body tracker to articulated objects and mechanisms containing:

- multiple rigid links;
- revolute, prismatic, spherical, and floating joints;
- kinematic trees;
- closed kinematic chains.

For $N$ bodies, the tracker estimates

$$
\mathcal X_k=
\left\{
{}^{C}T_{M_1,k},
\ldots,
{}^{C}T_{M_N,k}
\right\}
$$

from the current RGB-D data while requiring the body poses to remain kinematically consistent.

Conceptually,

$$
\hat{\mathcal X}_k
=
\arg\min_{\mathcal X\in\mathcal K}
\sum_{i=1}^{N}
E_i\left({}^{C}T_{M_i};D_k\right),
$$

where

- $D_k$ is the current observation;
- $E_i$ is the visual alignment energy of body $i$;
- $\mathcal K$ is the set of configurations satisfying the mechanism's joints and loop closures.

The central principle is

$$
\boxed{
\text{per-body visual evidence}
\rightarrow
\text{projection into joint coordinates}
\rightarrow
\text{closed-chain constraints}.
}
$$

ICG/ICG+ supplies the rigid-body visual evidence.
M3T supplies the multi-body kinematic optimization layer.

---

## 2. Local rigid-body pose evidence

For body $i$, define a small local pose correction

$$
\boldsymbol\theta_i=
\begin{bmatrix}
\boldsymbol\theta_{r,i}\\
\boldsymbol\theta_{t,i}
\end{bmatrix}
\in\mathbb R^6.
$$

Around the current pose,

$$
E_i(\boldsymbol\theta_i)
\approx
E_i(0)
+
\mathbf g_i^\top\boldsymbol\theta_i
+
\frac12
\boldsymbol\theta_i^\top H_i\boldsymbol\theta_i,
$$

where

$$
\mathbf g_i
=
\left.
\frac{\partial E_i}
{\partial\boldsymbol\theta_i}
\right|_{\boldsymbol\theta_i=0},
\qquad
H_i
=
\left.
\frac{\partial^2E_i}
{\partial\boldsymbol\theta_i^2}
\right|_{\boldsymbol\theta_i=0}.
$$

For an independent rigid object, the damped Newton step would be

$$
(H_i+\Sigma_i)\hat{\boldsymbol\theta}_i
=
-\mathbf g_i.
$$

The region and depth modalitiesâ€”and optionally the texture modality from ICG+â€”are accumulated as

$$
\mathbf g_i
=
\mathbf g_{r,i}
+
\mathbf g_{d,i}
+
\mathbf g_{t,i},
$$

$$
H_i
=
H_{r,i}
+
H_{d,i}
+
H_{t,i}.
$$

M3T does not apply these body corrections independently, because that would break the joints.
It first maps all body evidence into a common minimal kinematic coordinate system.

---

## 3. Minimal kinematic coordinates

Let the mechanism variation be

$$
\boldsymbol\theta_k=
\begin{bmatrix}
\boldsymbol\theta_{j_0}\\
\boldsymbol\theta_{j_1}\\
\vdots\\
\boldsymbol\theta_{j_{N-1}}
\end{bmatrix}
\in\mathbb R^{n_k}.
$$

Typical coordinate dimensions are:

| Joint | Minimal variation dimension |
|---|---:|
| Fixed | 0 |
| Revolute | 1 |
| Prismatic | 1 |
| Spherical | 3 |
| Floating root | 6 |

Each body variation is approximately related to the mechanism variation by

$$
\boxed{
\boldsymbol\theta_i
\approx
J_i\boldsymbol\theta_k,
}
\qquad
J_i=
\frac{\partial\boldsymbol\theta_i}
{\partial\boldsymbol\theta_k}
\in\mathbb R^{6\times n_k}.
$$

The total visual energy is

$$
E_k(\boldsymbol\theta_k)
=
\sum_{i=1}^{N}
E_i(J_i\boldsymbol\theta_k).
$$

Applying the chain rule gives the joint-space gradient

$$
\boxed{
\mathbf g_k
=
\sum_{i=1}^{N}
J_i^\top\mathbf g_i
}
$$

and, neglecting second derivatives of the kinematic mapping, the approximate joint-space Hessian

$$
\boxed{
H_k
\approx
\sum_{i=1}^{N}
J_i^\top H_iJ_i.
}
$$

This is the key mathematical bridge between rigid tracking and articulated tracking.
Each link is evaluated by an ordinary six-dimensional visual tracker, but its evidence is accumulated in the smaller, mechanically valid joint-coordinate space.

A visible link can therefore provide information about its ancestors and help estimate joints whose other descendant links are partially occluded.

---

## 4. Recursive body Jacobians

M3T uses adjoint transformations to express local variations in the appropriate frames.
For

$$
{}^{A}T_B=
\begin{bmatrix}
{}^{A}R_B & {}^{A}t_B\\
0 & 1
\end{bmatrix},
$$

the paper's rotation-translation ordering gives

$$
\operatorname{Ad}({}^{A}T_B)
=
\begin{bmatrix}
{}^{A}R_B & 0\\
[{}^{A}t_B]_\times{}^{A}R_B & {}^{A}R_B
\end{bmatrix}.
$$

Let body $i$ have parent $p(i)$, joint frame $J_i$, and model frame $M_i$.
Its variation results from:

1. the motion inherited from its parent;
2. its own joint variation.

Let $S_i$ embed the minimal joint coordinate into a six-dimensional local variation:

$$
\bar{\boldsymbol\theta}_{j_i}=S_i\boldsymbol\theta_{j_i}.
$$

For a revolute $z$-axis joint,

$$
\bar{\boldsymbol\theta}_{j_i}
=
\begin{bmatrix}
0&0&\theta_{j_i}&0&0&0
\end{bmatrix}^{\top}.
$$

The recursive body Jacobian has the structure

$$
J_i
=
\operatorname{Ad}
\left({}^{M_i}T_{P_i}\right)
J_{p(i)}
+
\begin{bmatrix}
0&\cdots&
\operatorname{Ad}
\left({}^{M_i}T_{J_i}\right)S_i
&\cdots&0
\end{bmatrix}.
$$

Thus, each joint affects its own body and all descendants.
For a pure tree, this minimal parameterization automatically guarantees joint consistency; the body poses cannot move independently and violate their connections.

---

## 5. Why closed chains require explicit constraints

A tree permits only one parent for each body.
A four-bar linkage, parallel gripper, or other closed mechanism contains extra relationships that close kinematic loops.

M3T handles these mechanisms by:

1. representing a spanning portion of the mechanism as a tree;
2. introducing explicit equality constraints for the remaining loop-closing edges.

For constraint frames $A$ and $B$ attached to bodies $a$ and $b$, compute

$$
{}^{A}T_B=
\begin{bmatrix}
{}^{A}R_B & {}^{A}t_B\\
0 & 1
\end{bmatrix}.
$$

Represent their relative rotation by

$$
{}^{A}R_B
=
\exp\left([{}^{A}\mathbf r_B]_\times\right).
$$

The full relative-pose error is

$$
\bar{\mathbf b}_{ab}
=
\begin{bmatrix}
{}^{A}\mathbf r_B\\
{}^{A}\mathbf t_B
\end{bmatrix}.
$$

The mechanism selects only the rows that must be constrained.
Examples:

- A rigid closure constrains all six components:

$$
\mathbf b_{ab}
=
\bar{\mathbf b}_{ab}
=0.
$$

- A closure that permits rotation around $z$ but locks the remaining directions can use

$$
\mathbf b_{ab}
=
\begin{bmatrix}
r_x&r_y&t_x&t_y&t_z
\end{bmatrix}^{\top}
=0.
$$

The constraint Jacobian with respect to the mechanism coordinates is

$$
B_{ab}
=
\frac{\partial\mathbf b_{ab}}
{\partial\boldsymbol\theta_k}
=
\frac{\partial\mathbf b_{ab}}
{\partial\boldsymbol\theta_a}J_a
+
\frac{\partial\mathbf b_{ab}}
{\partial\boldsymbol\theta_b}J_b.
$$

All loop constraints are stacked:

$$
\mathbf b_k=
\begin{bmatrix}
\mathbf b_1\\
\vdots\\
\mathbf b_m
\end{bmatrix},
\qquad
B_k=
\begin{bmatrix}
B_1\\
\vdots\\
B_m
\end{bmatrix}.
$$

---

## 6. Constrained Newton optimization

Introduce Lagrange multipliers $\boldsymbol\lambda$:

$$
\mathcal L(\boldsymbol\theta_k,\boldsymbol\lambda)
=
E_k(\boldsymbol\theta_k)
+
\mathbf b_k(\boldsymbol\theta_k)^\top
\boldsymbol\lambda.
$$

Linearize the gradient and constraints:

$$
\mathbf g_k(\boldsymbol\theta_k)
\approx
\mathbf g_k+H_k\boldsymbol\theta_k,
$$

$$
\mathbf b_k(\boldsymbol\theta_k)
\approx
\mathbf b_k+B_k\boldsymbol\theta_k.
$$

The constrained Newton step is obtained from the Karush-Kuhn-Tucker system

$$
\boxed{
\begin{bmatrix}
H_k+\Sigma & B_k^\top\\
B_k & 0
\end{bmatrix}
\begin{bmatrix}
\hat{\boldsymbol\theta}_k\\
\hat{\boldsymbol\lambda}
\end{bmatrix}
=
-
\begin{bmatrix}
\mathbf g_k\\
\mathbf b_k
\end{bmatrix}.
}
$$

The first block row,

$$
(H_k+\Sigma)\hat{\boldsymbol\theta}_k
+
B_k^\top\hat{\boldsymbol\lambda}
=
-\mathbf g_k,
$$

finds a visually favorable update while accounting for the constraint reactions.
The second block row,

$$
B_k\hat{\boldsymbol\theta}_k
=
-\mathbf b_k,
$$

moves the mechanism back toward the constraint manifold.

The paper uses pivoted Cholesky because the visual Hessian and constraint Jacobian may have very different numerical scales.

---

## 7. Rotation-constraint differential

The derivative of the $SO(3)$ logarithm is nonlinear.
Let

$$
{}^{A}\mathbf r_B=\alpha\mathbf e,
\qquad
\|\mathbf e\|=1.
$$

M3T defines the variation matrix

$$
C(\alpha,\mathbf e)
=
\frac{\alpha}{2}
\cot\left(\frac{\alpha}{2}\right)I
-
\frac{\alpha}{2}[\mathbf e]_\times
+
\left[
1-
\frac{\alpha}{2}
\cot\left(\frac{\alpha}{2}\right)
\right]\mathbf e\mathbf e^\top.
$$

This matrix represents the differential of the rotation-vector/logarithm mapping.
Near zero relative rotation,

$$
\lim_{\alpha\rightarrow0}C(\alpha,\mathbf e)=I.
$$

For the complete six-dimensional constraint, the derivatives with respect to body variations are

$$
\frac{\partial\bar{\mathbf b}_{ab}}
{\partial\boldsymbol\theta_a}
=
\begin{bmatrix}
-C\,{}^{A}R_{M_a} & 0\\
{}^{A}R_{M_a}[{}^{M_a}t_B]_\times
&
-{}^{A}R_{M_a}
\end{bmatrix},
$$

$$
\frac{\partial\bar{\mathbf b}_{ab}}
{\partial\boldsymbol\theta_b}
=
\begin{bmatrix}
C\,{}^{A}R_{M_b} & 0\\
-{}^{A}R_{M_b}[{}^{M_b}t_B]_\times
&
{}^{A}R_{M_b}
\end{bmatrix}.
$$

The upper blocks describe how relative rotation changes.
The lower-left blocks capture apparent translation caused by rotating around offset constraint frames, while the lower-right blocks describe direct translation.

---

## 8. Meaning of one-iteration constraint convergence

For a fully constrained relative rotation, the method constructs the relative change

$$
\Delta R
=
\exp\left(-[{}^{A}\mathbf r_B]_\times\right)
=
({}^{A}R_B)^{-1}.
$$

Consequently,

$$
\Delta R\,{}^{A}R_B=I.
$$

For translation,

$$
\Delta\mathbf t=-{}^{A}\mathbf t_B,
$$

so

$$
{}^{A}\mathbf t_B+\Delta\mathbf t=0.
$$

The correction can be distributed between the two participating bodies according to their visual Hessians, while the resulting relative correction cancels the closure error.

However,

$$
\boxed{
\text{one-step constraint convergence}
\ne
\text{one-step visual tracking convergence}.
}
$$

The nonlinear region/depth/texture correspondence problem still requires multiple iterations.
The one-step statement concerns the kinematic constraint, not complete image alignment.

---

## 9. Multi-body occlusion handling

Independent rigid-body trackers can incorrectly match geometry hidden behind another link.
M3T renders the complete mechanism to validate correspondences.

For depth:

- every body receives a distinct rendered ID;
- a surface point is used only if it projects onto that body's visible silhouette;
- samples hidden by another link are rejected.

For region tracking:

- links with similar color statistics may share a region ID;
- a contour correspondence is rejected when another silhouette interrupts its valid inside/outside interval;
- histogram updates use only pixels assigned to the correct rendered region.

The number of correspondence samples is scaled with each body's visible surface area and contour length.
This keeps evidence balanced among differently sized and differently occluded links.

---

## 10. Complete per-frame algorithm

For frame $k$:

1. initialize the mechanism from frame $k-1$;
2. render the current multi-body configuration;
3. establish valid region, depth, and optional texture correspondences;
4. compute $(\mathbf g_i,H_i)$ for each body;
5. recursively construct the body Jacobians $J_i$;
6. assemble

$$
\mathbf g_k=\sum_iJ_i^\top\mathbf g_i,
\qquad
H_k=\sum_iJ_i^\top H_iJ_i;
$$

7. calculate loop errors $\mathbf b_k$ and their Jacobian $B_k$;
8. solve the constrained KKT system;
9. update the root and minimal joint coordinates;
10. recover all child poses by forward kinematics;
11. regenerate correspondences and repeat the refinement.

The updated child pose has the forward-kinematic form

$$
{}^{A}T_{M_i}^{+}
=
{}^{A}T_{P_i}^{+}
{}^{P_i}T_{J_i}
T(\bar{\hat{\boldsymbol\theta}}_{j_i})
{}^{J_i}T_{M_i}.
$$

---

## 11. Reduction to a single rigid object

For one independent rigid object,

$$
N=1,
\qquad
J_1=I_6,
\qquad
B_k=\varnothing.
$$

The M3T optimization reduces to

$$
(H+\Sigma)\hat{\boldsymbol\theta}
=
-\mathbf g,
$$

followed by

$$
{}^{C}T_O^{+}
=
{}^{C}T_O\,T(\hat{\boldsymbol\theta}).
$$

Therefore, the multi-body Jacobian and KKT machinery matters only when tracking connected links or mechanisms.
For a single object, the relevant pipeline is simply

$$
\boxed{
\text{region/depth/texture correspondences}
\rightarrow
\text{Newton refinement}
\rightarrow
\text{pose update}.
}
$$

---

## 12. Practical interpretation and limitations

$$
\boxed{
\text{M3T is a fast constrained measurement optimizer, not a complete temporal state estimator.}
}
$$

Its kinematic model relates bodies at the **same timestep**.
The paper does not provide:

- a Kalman filter or fixed-lag smoother;
- object velocity or acceleration estimation;
- a constant-velocity process model;
- global pose initialization;
- guaranteed recovery after complete tracking loss.

The current frame normally starts from

$$
T_k^{(0)}=T_{k-1}^{+}.
$$

As a result, the system can still fail due to:

- large inter-frame motion or poor initialization;
- complete or prolonged occlusion;
- inaccurate metric meshes or camera calibration;
- symmetric geometry;
- weak foreground/background color separation;
- unreliable depth on reflective, transparent, or thin objects.

For an eye-in-hand camera, robot forward kinematics should predict the current camera-frame pose before M3T refinement:

$$
{}^{C_k}\hat T_{O,k}^{-}
=
\left({}^{B}T_C(q_k)\right)^{-1}
{}^{B}\hat T_{O,k}^{-}.
$$

A complete robot perception stack may therefore use

$$
\boxed{
\text{global initializer}
\rightarrow
\text{M3T/ICG+ local refinement}
\rightarrow
SE(3)\text{ temporal fusion with FK/contact}.
}
$$

## References

- [M. Stoiber et al., A Multi-body Tracking Framework - From Rigid Objects to Kinematic Structures, 2023](https://arxiv.org/pdf/2208.01502)
- [M. Stoiber et al., ICG+: A Multi-Modal Approach for 6DoF Object Tracking, 2023](https://arxiv.org/pdf/2302.11458)
- [M. Stoiber et al., Iterative Corresponding Geometry: Fusing Region and Depth for Highly Efficient 3D Tracking, CVPR 2022](https://elib.dlr.de/189883/1/Stoiber_Iterative_Corresponding_Geometry_Fusing_Region_and_Depth_for_Highly_Efficient_CVPR_2022_paper.pdf)
- [Official DLR 3D object-tracking repository](https://github.com/DLR-RM/3DObjectTracking)
