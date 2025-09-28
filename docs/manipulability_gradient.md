# Manipulability Gradient (PoE, Body/End–Effector Jacobian)

This note summarizes formulas and a compact derivation for the gradient of Yoshikawa’s manipulability measure when the Jacobian is built via the Product of Exponentials (PoE) in the body/end–effector frame. It matches standard Lie–group identities and is equivalent to product‑rule derivations written directly with exponentials.

## Notation

- Joint vector: $\theta = [\theta_1,\ldots,\theta_n]^T$.
- Body/end–effector Jacobian (6×n): $J_b(\theta) = [J_1\;\cdots\;J_n]$ with columns $J_i \in \mathbb{R}^6$.
- Yoshikawa manipulability: $\displaystyle w(\theta) = \sqrt{\det\big(J_b J_b^T\big)}$.
- Hat/vee maps: $\hat{S} \in \mathfrak{se}(3)$ for $S\in\mathbb{R}^6$ (we use screw order $S=[v;\,\omega]$); commutator $[A,B]=AB-BA$.
- Adjoint/adjoint: $\mathrm{Ad}_T \in \mathbb{R}^{6\times6},\; \mathrm{ad}_S \in \mathbb{R}^{6\times6}$ with our $[v;\,\omega]$ ordering
  $$
  \mathrm{ad}_{S} = \begin{bmatrix} [\omega]_\times & [v]_\times \\
                                       0               & [\omega]_\times \end{bmatrix},\quad
  \widehat{S \times T} = [\hat{S},\hat{T}].
  $$

## General gradient of Yoshikawa’s $w$

Let $A(\theta) = J_b J_b^T \in \mathbb{R}^{6\times6}$. Then

$$
\frac{\partial w}{\partial \theta_k}
  = \frac{w}{2}\;\mathrm{tr}\!\left(A^{-1} \frac{\partial A}{\partial \theta_k}\right),
\qquad
\frac{\partial A}{\partial \theta_k} = \frac{\partial J_b}{\partial \theta_k} J_b^T + J_b \left(\frac{\partial J_b}{\partial \theta_k}\right)^T.
$$

Using symmetry of $A^{-1}$ and cyclic trace, this simplifies to the numerically convenient form

$$
\boxed{\;\frac{\partial w}{\partial \theta_k} = w\,\mathrm{Tr}\!\left(\frac{\partial J_b}{\partial \theta_k}\,J_b^{\dagger}\right).\;}
$$
Here $J_b^{\dagger}$ denotes the Moore–Penrose pseudoinverse (for full row rank, $J_b^{\dagger}=J_b^T (J_b J_b^T)^{-1}$; near singularities use a damped or SVD form).

An equivalent SVD expression (useful near rank change) for $J_b = U\Sigma V^T$ is
$$
\frac{\partial w}{\partial \theta_k} = w\sum_{r=1}^{m} \frac{1}{\sigma_r}\, u_r^T\,\frac{\partial J_b}{\partial \theta_k}\,v_r,\quad m=\mathrm{rank}(J_b).
$$

## PoE body Jacobian and its derivatives

For PoE in the body frame, the $i$‑th column is (Modern Robotics; MLS94)

$$
J_i(\theta) = \mathrm{Ad}_{\,e^{-\hat{S}_n\theta_n}\cdots e^{-\hat{S}_{i+1}\theta_{i+1}}} \, S_i.
$$

Define $M_i := e^{-\hat{S}_n\theta_n}\cdots e^{-\hat{S}_{i+1}\theta_{i+1}}$. Then $\hat{J}_i = M_i\,\hat{S}_i\,M_i^{-1}$. Differentiating w.r.t. $\theta_j$ yields two regimes:

1) If $j \le i$, $\, e^{-\hat{S}_j\theta_j}$ is not in $M_i$, so $\partial J_i/\partial\theta_j = 0$.

2) If $j>i$,
$$
\frac{\partial \hat{J}_i}{\partial \theta_j}
 = \underbrace{\left(\frac{\partial M_i}{\partial\theta_j}\right) \hat{S}_i M_i^{-1} + M_i\hat{S}_i\left(\frac{\partial M_i^{-1}}{\partial\theta_j}\right)}_{\text{product rule}}
 = \big[(\partial M_i/\partial\theta_j)M_i^{-1},\; \hat{J}_i\big].
$$

Factor $M_i = F\,e^{-\hat{S}_j\theta_j}G$ with $F=e^{-\hat{S}_n\theta_n}\cdots e^{-\hat{S}_{j+1}\theta_{j+1}}$ and $G=e^{-\hat{S}_{j-1}\theta_{j-1}}\cdots e^{-\hat{S}_{i+1}\theta_{i+1}}$. Then
$$
(\partial M_i/\partial\theta_j)M_i^{-1} = F(-\hat{S}_j)F^{-1} = -\,\widehat{\mathrm{Ad}_F S_j} = -\,\hat{J}_j.
$$

<details>
<summary>Details: show (∂M_i/∂θ_j) M_i^{-1} = F(−Ŝ_j)F^{-1} = − Ĵ_j</summary>

1. Only the $j$‑th exponential depends on $\theta_j$, so with $E_j:=e^{-\hat{S}_j\theta_j}$ and constants (w.r.t. $\theta_j$) $F, G$,
   $$M_i = F\,E_j\,G,\qquad \frac{\partial M_i}{\partial\theta_j} = F\,\frac{\partial E_j}{\partial\theta_j}\,G.$$
2. For a constant matrix $A$, $\frac{\partial}{\partial\theta}e^{A\theta}=A e^{A\theta}$. Hence $\frac{\partial E_j}{\partial\theta_j}=(-\hat{S}_j)E_j$ and
   $$\frac{\partial M_i}{\partial\theta_j} = F\,(-\hat{S}_j)\,E_j\,G.$$
3. Right‑multiply by $M_i^{-1}=G^{-1}E_j^{-1}F^{-1}$ and cancel:
   $$\begin{aligned}
   (\partial M_i/\partial\theta_j)M_i^{-1}
   &= F(-\hat{S}_j)E_j\,G\;G^{-1}E_j^{-1}F^{-1}\\
   &= F(-\hat{S}_j)\,\underbrace{E_jE_j^{-1}}_{I}\,F^{-1} = F(-\hat{S}_j)F^{-1}.
   \end{aligned}$$
4. Use the adjoint identity $(\mathrm{Ad}_T S)^{\wedge}=T\hat{S}T^{-1}$. With $T=F$,
   $$F(-\hat{S}_j)F^{-1}=-(\mathrm{Ad}_F S_j)^{\wedge}=-\,\widehat{\mathrm{Ad}_F S_j}.$$
5. For the body Jacobian, $J_j(\theta)=\mathrm{Ad}_F S_j$, so $\widehat{\mathrm{Ad}_F S_j}=\hat{J}_j$ and the chain closes.

</details>

Therefore
$$
\boxed{\;\frac{\partial \hat{J}_i}{\partial \theta_j} = -\,[\hat{J}_j,\hat{J}_i] \;\;\Leftrightarrow\;\; \frac{\partial J_i}{\partial \theta_j} = -\,\mathrm{ad}_{J_j}\,J_i,\quad j>i;\qquad \frac{\partial J_i}{\partial \theta_j}=0,\; j\le i.\;}
$$

These are the standard “triangular” derivative identities for the body Jacobian. For the space Jacobian the signs/inequalities swap: $\partial J^s_i/\partial\theta_j = \mathrm{ad}_{J^s_j} J^s_i$ for $j<i$, zero otherwise.

<details>
<summary>Derivation: product‑rule terms become a commutator</summary>

For any differentiable, invertible matrix $M(\theta)$ and constant matrix $X$,
$$
\frac{\partial}{\partial\theta}\big(M X M^{-1}\big)
 = (\partial M) X M^{-1} + M X (\partial M^{-1}).
$$
Use the inverse derivative identity, obtained from $I = M M^{-1}$:
$$
0 = (\partial M)M^{-1} + M(\partial M^{-1})\;\;\Rightarrow\;\; \partial M^{-1} = -\,M^{-1}(\partial M)M^{-1}.
$$
Substitute and define $Q := (\partial M)M^{-1}$:
$$
\begin{aligned}
\frac{\partial}{\partial\theta}\big(M X M^{-1}\big)
&= (\partial M) X M^{-1} - M X M^{-1} (\partial M) M^{-1}\\
&= Q\,(M X M^{-1}) - (M X M^{-1})\,Q\\
&= \big[\,Q,\; M X M^{-1}\,\big].
\end{aligned}
$$
Applying this with $M=M_i$, $X=\hat{S}_i$ and $M X M^{-1} = \hat{J}_i$ gives
$$
\frac{\partial \hat{J}_i}{\partial \theta_j} = \big[\,(\partial M_i/\partial\theta_j) M_i^{-1},\; \hat{J}_i\,\big].
$$
In our PoE setting, $(\partial M_i/\partial\theta_j) M_i^{-1} = -\hat{J}_j$ for $j>i$, recovering the compact identity above. Note this derivation is purely algebraic (derivative of a similarity transform) and does not rely on Lie‑group specifics, though here $Q\in\mathfrak{se}(3)$.

</details>

## Assembling the manipulability gradient

Let the manipulator Hessian be the third‑order tensor $\mathcal{H} \in \mathbb{R}^{6\times n\times n}$ with components
$$
\mathcal{H}_{a i k} := \frac{\partial (J_b)_{a i}}{\partial \theta_k},\quad a\in\{1,\dots,6\},\; i,k\in\{1,\dots,n\}.
$$
Denote the $k$‑th slice along the third index by $H^{(k)} := \partial J_b/\partial\theta_k \in \mathbb{R}^{6\times n}$. From the body identities above,
$$
H^{(k)} = \big[\,-\,\mathrm{ad}_{J_k}J_1\;\cdots\;-\,\mathrm{ad}_{J_k}J_{k-1}\;\;0\;\cdots\;0\big].
$$
The gradient component is
$$
\boxed{\;\frac{\partial w}{\partial \theta_k} = w\,\mathrm{Tr}(H^{(k)} \, J_b^{\dagger})\;}
$$
and using the triangular structure,
$$
\mathrm{Tr}(H^{(k)} J_b^{\dagger}) = -\sum_{i=1}^{k-1} (J_b^{\dagger})_{i,:}\,\big(\mathrm{ad}_{J_k}J_i\big).
$$
Here $(J_b^{\dagger})_{i,:}\in\mathbb{R}^{1\times 6}$ is row $i$ of $J_b^{\dagger}$. The dot denotes the standard Euclidean inner product in $\mathbb{R}^6$.

## Equivalence to product‑rule expansions

If you expand $\partial J_i/\partial\theta_j$ by differentiating the chain of exponentials (as in handwritten notes), you obtain two terms—one from $\partial M_i$ and one from $\partial M_i^{-1}$. Using $\partial M^{-1} = -M^{-1}(\partial M)M^{-1}$, these combine into a commutator
$$
\frac{\partial \hat{J}_i}{\partial \theta_j} = \big[(\partial M_i/\partial\theta_j)M_i^{-1},\; \hat{J}_i\big],
$$
which, after factoring out the $j$‑th exponential, reduces to $-\,[\hat{J}_j,\hat{J}_i]$. This is exactly the Lie‑bracket identity above. The overall sign is set by the body‑frame use of negative exponents in $M_i$.

## Practical notes

- Near singularities, $(J_b J_b^T)^{-1}$ may be ill‑conditioned; use a damped inverse $(J_b J_b^T + \lambda^2 I)^{-1}$ or SVD pseudoinverse in practice.
- Dimensions: $J_b\in\mathbb{R}^{6\times n}$, $J_b^{\dagger}\in\mathbb{R}^{n\times 6}$, $\mathcal{H}\in\mathbb{R}^{6\times n\times n}$ with slice $H^{(k)}\in\mathbb{R}^{6\times n}$; $\mathrm{Tr}(H^{(k)} J_b^{\dagger})$ is scalar.
- The body identities imply $\partial J_i/\partial\theta_i = 0$. For the space Jacobian, self‑derivatives are also zero but the triangular structure flips.

## References

- T. Yoshikawa, “Manipulability of Robotic Mechanisms,” IJRR, 1985.
- R. M. Murray, Z. Li, S. S. Sastry, A Mathematical Introduction to Robotic Manipulation, 1994.
- K. M. Lynch, F. C. Park, Modern Robotics: Mechanics, Planning, and Control, 2017.
- S. Chiaverini, “Singularity‑Robust Task‑Priority Redundancy Resolution,” 1997.
- J. Selig, Geometric Fundamentals of Robotics, 2005.
- V. Kurtz, P. M. Wensing, H. Lin, “Control Barrier Functions for Singularity Avoidance in Passivity‑Based Manipulator Control,” CDC 2021.
- G. Marani, J. Kim, J. Yuh, W. K. Chung, “A Real‑Time Approach for Singularity Avoidance in Resolved Motion Rate Control of Robotic Manipulators,” ICRA 2002.
