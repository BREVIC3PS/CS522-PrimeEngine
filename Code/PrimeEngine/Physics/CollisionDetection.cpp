#include "CollisionDetection.h"
#include <iostream>


#define GJK_MAX_ITERATIONS 128
#define GJK_ACCURACY ((float)0.0001)
#define GJK_MIN_DISTANCE ((float)0.0001)
#define GJK_DUPLICATED_EPS ((float)0.0001)
#define GJK_SIMPLEX2_EPS ((float)0.0)
#define GJK_SIMPLEX3_EPS ((float)0.0)
#define GJK_SIMPLEX4_EPS ((float)0.0)

#define EPA_ACCURACY ((float)0.0001)
#define EPA_PLANE_EPS ((float)0.00001)
#define EPA_INSIDE_EPS ((float)0.01)
#define EPA_MAX_VERTICES 128
#define EPA_MAX_ITERATIONS 255
#define EPA_FALLBACK (10 * EPA_ACCURACY)
#define EPA_MAX_FACES (EPA_MAX_VERTICES * 2)

typedef unsigned int U;
typedef unsigned char U1;
typedef PE::Components::MinkowskiDiff tShape;

namespace PE
{
    namespace Components
    {
        inline Vector3 MinkowskiDiff::Support1(Vector3& dir)
        {
           /* Matrix4x4 transform = box1->m_worldTransform;
            Matrix4x4 world2local = transform.inverse();
            Vector3 localDir = world2local * dir;
            Vector3 localSupportPoint = box1->getAABB()->LocalGetSupportVertex(localDir);
            Vector3 globalSupportPoint = transform *localSupportPoint;
            return globalSupportPoint;*/

            //Vector3 globalSupportPoint = box1->getAABB()->LocalGetSupportVertex(dir);
            Vector3 globalSupportPoint = box1->GetSupport(dir);
            return globalSupportPoint;
        }

        inline Vector3 MinkowskiDiff::Support2(Vector3& dir)
        {
           /* Matrix4x4 transform = box2->m_worldTransform;
            Matrix4x4 world2local = transform.inverse();
            Vector3 localDir = world2local * dir;
            Vector3 localSupportPoint = box2->getAABB()->LocalGetSupportVertex(localDir);
            Vector3 globalSupportPoint = transform * localSupportPoint;
            return globalSupportPoint;*/

            //Vector3 globalSupportPoint = box2->getAABB()->LocalGetSupportVertex(dir);
            Vector3 globalSupportPoint = box2->GetSupport(dir);
            return globalSupportPoint;
        }

        Vector3 MinkowskiDiff::Support(Vector3& dir)
        {
            Vector3 s1 = Support1(dir);
            Vector3 s2 = Support2(dir * -1);
            return s1 - s2;
        }

        Vector3 MinkowskiDiff::Support(Vector3& dir, int idx)
        {
            if (idx == 1)
            {
                return (Support1(dir));
            }
            else
            {
                return (Support2(dir));
            }
        }

        ////////////////////////////////////////////// GJK //////////////////////

        struct GJK
        {
            struct sSV
            {
                Vector3 d, w;
            };
            struct sSimplex
            {
                sSV* c[4];
                float p[4];
                U rank;
            };
            struct eStatus
            {
                enum _
                {
                    valid,
                    Inside,
                    Failed
                };
            };

            /* Fields */
            MinkowskiDiff m_shape;
            Vector3 m_ray;
            float m_distance;
            sSimplex m_simplices[2];
            sSV m_store[4];
            sSV* m_free[4];
            U m_nfree;
            U m_current;
            sSimplex* m_simplex;
            eStatus::_ m_status;

            /* Methods */
            GJK()
            {
                Initialize();
            }
            void Initialize()
            {
                m_ray = Vector3(0, 0, 0);
                m_nfree = 0;
                m_status = eStatus::Failed;
                m_current = 0;
                m_distance = 0;
            }
            eStatus::_ Evaluate(const tShape& shapearg, const Vector3& guess)
            {
                U iterations = 0;
                float sqdist = 0;
                float alpha = 0;
                Vector3 lastw[4];
                U clastw = 0;

                /* Initialize solver*/
                m_free[0] = &m_store[0];
                m_free[1] = &m_store[1];
                m_free[2] = &m_store[2];
                m_free[3] = &m_store[3];
                m_nfree = 4;
                m_current = 0;
                m_status = eStatus::valid;
                m_shape = shapearg;
                m_distance = 0;

                //Initialize simplex
                m_simplices[0].rank = 0;
                m_ray = guess;
                const float sqrl = m_ray.lengthSqr();
                appendvertice(m_simplices[0], sqrl > 0 ? m_ray * -1 : Vector3(1, 0, 0));
                m_simplices[0].p[0] = 1;
                m_ray = m_simplices[0].c[0]->w;
                sqdist = sqrl;
                lastw[0] =
                    lastw[1] =
                    lastw[2] =
                    lastw[3] = m_ray;

                /* Loop						*/
                do
                {
                    const U next = 1 - m_current;
                    sSimplex& cs = m_simplices[m_current];
                    sSimplex& ns = m_simplices[next];

                    //Check zero
                    const float rl = m_ray.length();
                    if (rl < GJK_MIN_DISTANCE)
                    {
                        // Touching or inside
                        m_status = eStatus::Inside;
                        break;
                    }

                    //Append new vertice in -'v' direction
                    appendvertice(cs, m_ray * -1);
                    const Vector3& w = cs.c[cs.rank - 1]->w;
                    bool found = false;
                    for (U i = 0; i < 4; i++)
                    {
                        Vector3 diff = w - lastw[i];
                        if (diff.lengthSqr() < GJK_DUPLICATED_EPS)
                        {
                            found = true;
                            break;
                        }
                    }
                    if (found)
                    {
                        /* Return old simplex				*/
                        removevertice(m_simplices[m_current]);
                        break;
                    }
                    else
                    {
                        /* Update lastw					*/
                        lastw[clastw = (clastw + 1) & 3] = w;
                    }
                    /* Check for termination				*/
                    const float omega = m_ray.dotProduct(w) / rl;
                    alpha = omega > alpha ? omega : alpha;
                    if (((rl - alpha) - (GJK_ACCURACY * rl)) <= 0)
                    {
                        //Return old simplex
                        removevertice(m_simplices[m_current]);
                        break;
                    }

                    //Reduce simplex
                    float weights[4];
                    U mask = 0;
                    switch (cs.rank)
                    {
                    case 2:
                        sqdist = projectorigin(cs.c[0]->w,
                            cs.c[1]->w,
                            weights, mask);
                        break;
                    case 3:
                        sqdist = projectorigin(cs.c[0]->w,
                            cs.c[1]->w,
                            cs.c[2]->w,
                            weights, mask);
                        break;
                    case 4:
                        sqdist = projectorigin(cs.c[0]->w,
                            cs.c[1]->w,
                            cs.c[2]->w,
                            cs.c[3]->w,
                            weights, mask);
                        break;
                    }
                    if (sqdist >= 0)
                    {
                        // Valid
                        ns.rank = 0;
                        m_ray = Vector3(0, 0, 0);
                        m_current = next;
                        for (U i = 0, ni = cs.rank; i < ni; i++)
                        {
                            if (mask & (1 << i))
                            {
                                ns.c[ns.rank] = cs.c[i];
                                ns.p[ns.rank++] = weights[i];
                                m_ray = m_ray + cs.c[i]->w * weights[i];

                            }
                            else
                            {
                                m_free[m_nfree++] = cs.c[i];
                            }
                        }
                        if (mask == 15) m_status = eStatus::Inside;
                    }
                    else
                    {
                        // Return old simplex
                        removevertice(m_simplices[m_current]);
                        break;
                    }
                    m_status = ((++iterations) < GJK_MAX_ITERATIONS) ? m_status : eStatus::Failed;
                } while (m_status == eStatus::valid);

                m_simplex = &m_simplices[m_current];
                switch (m_status)
                {
                case PE::Components::GJK::eStatus::valid:
                    m_distance = m_ray.length();
                    break;
                case PE::Components::GJK::eStatus::Inside:
                    m_distance = 0;
                    break;
                default:
                    break;
                }
                return m_status;
            }

            bool EncloseOrigin()
            {
                switch (m_simplex->rank)
                {
                case 1:
                {
                    for (U i = 0; i < 3; ++i)
                    {
                        Vector3 axis = Vector3(0, 0, 0);
                        axis.m_values[i] = 1;
                        appendvertice(*m_simplex, axis);
                        if (EncloseOrigin()) return (true);
                        removevertice(*m_simplex);
                        appendvertice(*m_simplex, axis * -1);
                        if (EncloseOrigin()) return (true);
                        removevertice(*m_simplex);
                    }
                }
                break;
                case 2:
                {
                    Vector3 d = m_simplex->c[1]->w - m_simplex->c[0]->w;
                    for (U i = 0; i < 3; ++i)
                    {
                        Vector3 axis = Vector3(0, 0, 0);
                        axis.m_values[i] = 1;
                        Vector3 p = d.crossProduct(axis);
                        if (p.lengthSqr() > 0)
                        {
                            appendvertice(*m_simplex, p);
                            if (EncloseOrigin()) return (true);
                            removevertice(*m_simplex);
                            appendvertice(*m_simplex, p * -1);
                            if (EncloseOrigin()) return (true);
                            removevertice(*m_simplex);
                        }
                    }
                }
                break;
                case 3:
                {
                    Vector3 n = (m_simplex->c[1]->w - m_simplex->c[0]->w).crossProduct(m_simplex->c[2]->w - m_simplex->c[0]->w);
                    if (n.lengthSqr() > 0)
                    {
                        appendvertice(*m_simplex, n);
                        if (EncloseOrigin()) return (true);
                        removevertice(*m_simplex);
                        appendvertice(*m_simplex, n * -1);
                        if (EncloseOrigin()) return (true);
                        removevertice(*m_simplex);
                    }
                }
                break;
                case 4:
                {
                    if (abs(det(m_simplex->c[0]->w - m_simplex->c[3]->w,
                        m_simplex->c[1]->w - m_simplex->c[3]->w,
                        m_simplex->c[2]->w - m_simplex->c[3]->w)) > 0)
                        return (true);
                }
                break;
                }
                return (false);
            }
            /* Internals	*/
            void getsupport(Vector3& d, sSV& sv)
            {
                sv.d = d * (1 / d.length());
                sv.w = m_shape.Support(sv.d);
            }
            void appendvertice(sSimplex& simplex, Vector3& v)
            {
                simplex.p[simplex.rank] = 0;
                simplex.c[simplex.rank] = m_free[--m_nfree];
                getsupport(v, *simplex.c[simplex.rank++]);
            }
            void removevertice(sSimplex& simplex)
            {
                m_free[m_nfree++] = simplex.c[--simplex.rank];
            }
            static float projectorigin(Vector3& a,
                Vector3& b,
                float* w, U& m)
            {
                Vector3 d = b - a;
                float l = d.lengthSqr();
                if (l > GJK_SIMPLEX2_EPS)
                {
                    const float t(l > 0 ? (-(a.dotProduct(d))) / l : 0);
                    if (t >= 1)
                    {
                        w[0] = 0;
                        w[1] = 1;
                        m = 2;
                        return (b.lengthSqr());
                    }
                    else if (t <= 0)
                    {
                        w[0] = 1;
                        w[1] = 0;
                        m = 1;
                        return (a.lengthSqr());
                    }
                    else
                    {
                        w[0] = 1 - (w[1] = t);
                        m = 3;
                        return ((a + d * t).lengthSqr());
                    }
                }
                return (-1);
            }
            static float projectorigin(Vector3& a,
                Vector3& b,
                Vector3& c,
                float* w, U& m)
            {
                static const U imd3[] = { 1, 2, 0 };
                Vector3* vt[] = { &a, &b, &c };
                Vector3 dl[] = { a - b, b - c, c - a };
                Vector3 n = dl[0].crossProduct(dl[1]);
                const float l = n.lengthSqr();
                if (l > GJK_SIMPLEX3_EPS)
                {
                    float mindist = -1;
                    float subw[2] = { 0.f, 0.f };
                    U subm(0);
                    for (U i = 0; i < 3; ++i)
                    {
                        if (vt[i]->dotProduct(dl[i].crossProduct(n)) > 0)
                        {
                            const U j = imd3[i];
                            const float subd(projectorigin(*vt[i], *vt[j], subw, subm));
                            if ((mindist < 0) || (subd < mindist))
                            {
                                mindist = subd;
                                m = static_cast<U>(((subm & 1) ? 1 << i : 0) + ((subm & 2) ? 1 << j : 0));
                                w[i] = subw[0];
                                w[j] = subw[1];
                                w[imd3[j]] = 0;
                            }
                        }
                    }
                    if (mindist < 0)
                    {
                        const float d = a.dotProduct(n);
                        const float s = sqrt(l);
                        Vector3 p = n * (d / l);
                        mindist = p.lengthSqr();
                        m = 7;
                        w[0] = (dl[1].crossProduct(b - p).length() / s);
                        w[1] = (dl[2].crossProduct(c - p).length() / s);
                        w[2] = 1 - (w[0] + w[1]);
                    }
                    return (mindist);
                }
                return (-1);
            }
            static float projectorigin(Vector3& a,
                Vector3& b,
                Vector3& c,
                Vector3& d,
                float* w, U& m)
            {
                static const U imd3[] = { 1, 2, 0 };
                Vector3* vt[] = { &a, &b, &c, &d };
                Vector3 dl[] = { a - d, b - d, c - d };
                float vl = det(dl[0], dl[1], dl[2]);
                bool ng = (vl * a.dotProduct((b - c).crossProduct(a - b))) <= 0;
                if (ng && (abs(vl) > GJK_SIMPLEX4_EPS))
                {
                    float mindist = -1;
                    float subw[3] = { 0.f, 0.f, 0.f };
                    U subm(0);
                    for (U i = 0; i < 3; ++i)
                    {
                        const U j = imd3[i];
                        const float s = vl * d.dotProduct((dl[i].crossProduct(dl[j])));
                        if (s > 0)
                        {
                            const float subd = projectorigin(*vt[i], *vt[j], d, subw, subm);
                            if ((mindist < 0) || (subd < mindist))
                            {
                                mindist = subd;
                                m = static_cast<U>((subm & 1 ? 1 << i : 0) +
                                    (subm & 2 ? 1 << j : 0) +
                                    (subm & 4 ? 8 : 0));
                                w[i] = subw[0];
                                w[j] = subw[1];
                                w[imd3[j]] = 0;
                                w[3] = subw[2];
                            }
                        }
                    }
                    if (mindist < 0)
                    {
                        mindist = 0;
                        m = 15;
                        w[0] = det(c, b, d) / vl;
                        w[1] = det(a, c, d) / vl;
                        w[2] = det(b, a, d) / vl;
                        w[3] = 1 - (w[0] + w[1] + w[2]);
                    }
                    return (mindist);
                }
                return (-1);
            }
            static float det(const Vector3& a, const Vector3& b, const Vector3& c)
            {
                return (a.m_y * b.m_z * c.m_x + a.m_z * b.m_x * c.m_y -
                    a.m_x * b.m_z * c.m_y - a.m_y * b.m_x * c.m_z +
                    a.m_x * b.m_y * c.m_z - a.m_z * b.m_y * c.m_x);
            }
        };
            template < typename T>
            void btSwap(T& a, T& b)
            {
                T tmp = a;
                a = b;
                b = tmp;
            }

        ////////////////////////////////////////////// GJK //////////////////////


        ////////////////////////////////////////////// EPA //////////////////////

        struct EPA
        {
            /* Types */
            typedef GJK::sSV sSV;
            struct sFace
            {
                Vector3 n;
                float d;
                sSV* c[3];
                sFace* f[3];
                sFace* l[2];
                U1 e[3];
                U1 pass;
            };
            struct sList
            {
                sFace* root;
                U count;
                sList() : root(0),count(0){}
            };
            struct sHorizon
            {
                sFace* cf;
                sFace* ff;
                U nf;
                sHorizon(): cf(0),ff(0),nf(0){}
            };
            struct eStatus
            {
                enum _ 
                {
                    Valid,
                    Touching,
                    Degenerated,
                    NonConvex,
                    InvalidHull,
                    OutOfFaces,
                    OutOfVertices,
                    AccuraryReached,
                    FallBack,
                    Failed
                };
            };

            /* Fields */
            eStatus::_ m_status;
            GJK::sSimplex m_result;
            Vector3 m_normal;
            float m_depth;
            sSV m_sv_store[EPA_MAX_VERTICES];
            sFace m_fc_store[EPA_MAX_FACES];
            U m_nextsv;
            sList m_hull;
            sList m_stock;
            /* Methods		*/
            EPA()
            {
                Initialize();
            }

            static inline void bind(sFace* fa, U ea, sFace* fb, U eb)
            {
                fa->e[ea] = (U1)eb;
                fa->f[ea] = fb;
                fb->e[eb] = (U1)ea;
                fb->f[eb] = fa;
            }
            static inline void append(sList& list, sFace* face)
            {
                face->l[0] = 0;
                face->l[1] = list.root;
                if (list.root) list.root->l[0] = face;
                list.root = face;
                ++list.count;
            }
            static inline void remove(sList& list, sFace* face)
            {
                if (face->l[1]) face->l[1]->l[0] = face->l[0];
                if (face->l[0]) face->l[0]->l[1] = face->l[1];
                if (face == list.root) list.root = face->l[1];
                --list.count;
            }

            void Initialize()
            {
                m_status = eStatus::Failed;
                m_normal = Vector3(0, 0, 0);
                m_depth = 0;
                m_nextsv = 0;
                for (U i = 0; i < EPA_MAX_FACES; ++i)
                {
                    append(m_stock, &m_fc_store[EPA_MAX_FACES - i - 1]);
                }
            }

            eStatus::_ Evaluate(GJK& gjk, Vector3& guess)
            {
                GJK::sSimplex& simplex = *gjk.m_simplex;
                if ((simplex.rank > 1) && gjk.EncloseOrigin())
                {
                    /* Clean up				*/
                    while (m_hull.root)
                    {
                        sFace* f = m_hull.root;
                        remove(m_hull, f);
                        append(m_stock, f);
                    }
                    m_status = eStatus::Valid;
                    m_nextsv = 0;
                    /* Orient simplex		*/
                    if (gjk.det(simplex.c[0]->w - simplex.c[3]->w,
                        simplex.c[1]->w - simplex.c[3]->w,
                        simplex.c[2]->w - simplex.c[3]->w) < 0)
                    {
                        btSwap(simplex.c[0], simplex.c[1]);
                        btSwap(simplex.p[0], simplex.p[1]);
                    }
                    /* Build initial hull	*/
                    sFace* tetra[] = { newface(simplex.c[0], simplex.c[1], simplex.c[2], true),
                                      newface(simplex.c[1], simplex.c[0], simplex.c[3], true),
                                      newface(simplex.c[2], simplex.c[1], simplex.c[3], true),
                                      newface(simplex.c[0], simplex.c[2], simplex.c[3], true) };
                    if (m_hull.count == 4)
                    {
                        sFace* best = findbest();
                        sFace outer = *best;
                        U pass = 0;
                        U iterations = 0;
                        bind(tetra[0], 0, tetra[1], 0);
                        bind(tetra[0], 1, tetra[2], 0);
                        bind(tetra[0], 2, tetra[3], 0);
                        bind(tetra[1], 1, tetra[3], 2);
                        bind(tetra[1], 2, tetra[2], 1);
                        bind(tetra[2], 2, tetra[3], 1);
                        m_status = eStatus::Valid;

                        for (; iterations < EPA_MAX_ITERATIONS; ++iterations)
                        {
                            if (m_nextsv < EPA_MAX_VERTICES)
                            {
                                sHorizon horizon;
                                sSV* w = &m_sv_store[m_nextsv++];
                                bool valid = true;
                                best->pass = (U1)(++pass);
                                gjk.getsupport(best->n, *w);
                                const float wdist = best->n.dotProduct(w->w) - best->d;
                                if (wdist > EPA_ACCURACY)
                                {
                                    for (U j = 0; (j < 3) && valid; ++j)
                                    {
                                        valid &= expand(pass, w,
                                            best->f[j], best->e[j],
                                            horizon);
                                    }
                                    if (valid && (horizon.nf >= 3))
                                    {
                                        bind(horizon.cf, 1, horizon.ff, 2);
                                        remove(m_hull, best);
                                        append(m_stock, best);
                                        best = findbest();
                                        outer = *best;
                                    }
                                    else
                                    {
                                        m_status = eStatus::InvalidHull;
                                        break;
                                    }
                                }
                                else
                                {
                                    m_status = eStatus::AccuraryReached;
                                    break;
                                }
                            }
                            else
                            {
                                m_status = eStatus::OutOfVertices;
                                break;
                            }
                        }

                        const Vector3 projection = outer.n * outer.d;
                        m_normal = outer.n;
                        m_depth = outer.d;
                        m_result.rank = 3;
                        m_result.c[0] = outer.c[0];
                        m_result.c[1] = outer.c[1];
                        m_result.c[2] = outer.c[2];
                        m_result.p[0] = (outer.c[1]->w - projection).crossProduct(
                            outer.c[2]->w - projection)
                            .length();
                        m_result.p[1] = (outer.c[2]->w - projection).crossProduct(
                            outer.c[0]->w - projection)
                            .length();
                        m_result.p[2] = (outer.c[0]->w - projection).crossProduct(
                            outer.c[1]->w - projection)
                            .length();
                        const float sum = m_result.p[0] + m_result.p[1] + m_result.p[2];
                        m_result.p[0] /= sum;
                        m_result.p[1] /= sum;
                        m_result.p[2] /= sum;
                        return (m_status);
                    }
                }

                /* Fallback		*/
                m_status = eStatus::FallBack;
                m_normal = guess * -1;
                const float nl = m_normal.length();
                if (nl > 0)
                    m_normal = m_normal * (1 / nl);
                else
                    m_normal = Vector3(1, 0, 0);
                m_depth = 0;
                m_result.rank = 1;
                m_result.c[0] = simplex.c[0];
                m_result.p[0] = 1;
                return (m_status);
            }

            sFace* findbest()
            {
                sFace* minf = m_hull.root;
                float mind = minf->d * minf->d;
                for (sFace* f = minf->l[1]; f; f = f->l[1])
                {
                    const float sqd = f->d * f->d;
                    if (sqd < mind)
                    {
                        minf = f;
                        mind = sqd;
                    }
                }
                return (minf);
            }

            bool getedgedist(sFace* face, sSV* a, sSV* b, float& dist)
            {
                Vector3 ba = b->w - a->w;
                Vector3 n_ab = ba.crossProduct( face->n);   // Outward facing edge normal direction, on triangle plane
                float a_dot_nab = a->w.dotProduct(n_ab);  // Only care about the sign to determine inside/outside, so not normalization required

                if (a_dot_nab < 0)
                {
                    // Outside of edge a->b

                    const float ba_l2 = ba.lengthSqr();
                    const float a_dot_ba = a->w.dotProduct(ba);
                    const float b_dot_ba = b->w.dotProduct(ba);

                    if (a_dot_ba > 0)
                    {
                        // Pick distance vertex a
                        dist = a->w.length();
                    }
                    else if (b_dot_ba < 0)
                    {
                        // Pick distance vertex b
                        dist = b->w.length();
                    }
                    else
                    {
                        // Pick distance to edge a->b
                        const float a_dot_b = a->w.dotProduct(b->w);

                        float t = (a->w.lengthSqr() * b->w.lengthSqr() - a_dot_b * a_dot_b) / ba_l2;
                        float bigger = t >= 0 ? t : 0;
                        dist = sqrtf(bigger);
                    }

                    return true;
                }

                return false;
            }

            sFace* newface(sSV* a, sSV* b, sSV* c, bool forced)
            {
                if (m_stock.root)
                {
                    sFace* face = m_stock.root;
                    remove(m_stock, face);
                    append(m_hull, face);
                    face->pass = 0;
                    face->c[0] = a;
                    face->c[1] = b;
                    face->c[2] = c;
                    face->n = (b->w - a->w).crossProduct( c->w - a->w);

                    const float l = face->n.length();
                    const bool v = l > EPA_ACCURACY;

                    if (v)
                    {
                        if (!(getedgedist(face, a, b, face->d) ||
                            getedgedist(face, b, c, face->d) ||
                            getedgedist(face, c, a, face->d)))
                        {
                            // Origin projects to the interior of the triangle
                            // Use distance to triangle plane
                            face->d = a->w.dotProduct(face->n) / l;
                        }

                        face->n = face->n * (1 / l);
                        if (forced || (face->d >= -EPA_PLANE_EPS))
                        {
                            return face;
                        }
                        else
                            m_status = eStatus::NonConvex;
                    }
                    else
                        m_status = eStatus::Degenerated;

                    remove(m_hull, face);
                    append(m_stock, face);
                    return 0;
                }
                m_status = m_stock.root ? eStatus::OutOfVertices : eStatus::OutOfFaces;
                return 0;
            }

            bool expand(U pass, sSV* w, sFace* f, U e, sHorizon& horizon)
            {
                static const U i1m3[] = { 1, 2, 0 };
                static const U i2m3[] = { 2, 0, 1 };
                if (f->pass != pass)
                {
                    const U e1 = i1m3[e];
                    if ((f->n.dotProduct(w->w) - f->d) < -EPA_PLANE_EPS)
                    {
                        sFace* nf = newface(f->c[e1], f->c[e], w, false);
                        if (nf)
                        {
                            bind(nf, 0, f, e);
                            if (horizon.cf)
                                bind(horizon.cf, 1, nf, 2);
                            else
                                horizon.ff = nf;
                            horizon.cf = nf;
                            ++horizon.nf;
                            return (true);
                        }
                    }
                    else
                    {
                        const U e2 = i2m3[e];
                        f->pass = (U1)pass;
                        if (expand(pass, w, f->f[e1], f->e[e1], horizon) &&
                            expand(pass, w, f->f[e2], f->e[e2], horizon))
                        {
                            remove(m_hull, f);
                            append(m_stock, f);
                            return (true);
                        }
                    }
                }
                return (false);
            }
        };

        ////////////////////////////////////////////// EPA //////////////////////


        void CollisionDetector::CollideDetection(PhysicsShape* Shape1, PhysicsShape* Shape2, std::vector<std::shared_ptr<ContactManifold>>& collisions)
        {
            Vector3 position1 = Shape1->GetPosition();
            Vector3 position2 = Shape2->GetPosition();
            Vector3 guess = position1 - position2;
            sResults result;
            if (Penetration(Shape1, Shape2, guess, result))
            {
                ContactPoint point;
                point.localPositionA = result.witnessesInFirstLocal[0];
                point.localPositionB = result.witnessesInFirstLocal[1];
                point.globalPositionA = result.witnessInGlobal[0];
                point.globalPositionB = result.witnessInGlobal[1];
                point.normal = result.normal;
                point.penetrationDistance = result.distance;
                GenerateTangents(point);

                point.rA = point.globalPositionA - position1;
                point.rB = point.globalPositionB - position2;

                std::shared_ptr<ContactManifold> manifold = std::make_shared<ContactManifold>();
                manifold->colliderA = Shape1;
                manifold->colliderB = Shape2;
                manifold->contactPoints[0] = point;
                manifold->contactPointCount = 1;

                collisions.push_back(manifold);


            }
            
        }

        void CollisionDetector::InitializeMinkowskiDiff(PhysicsShape* Shape1, PhysicsShape* Shape2, sResults& result, MinkowskiDiff& diff)
        {
            // result
            result.witnessesInFirstLocal[0] = result.witnessesInFirstLocal[1] = result.witnessInGlobal[0] = result.witnessInGlobal[1] = Vector3(0, 0, 0);
            result.status = sResults::Separated;

            //minkowski
            diff.box1 = Shape1;
            diff.box2 = Shape2;
        }

        bool CollisionDetector::Penetration(PhysicsShape* Shape1, PhysicsShape* Shape2, Vector3& guess, sResults& result)
        {
            MinkowskiDiff shape;
            InitializeMinkowskiDiff(Shape1, Shape2, result, shape);

            // gjk 
            GJK gjk;
            GJK::eStatus::_ gjk_status = gjk.Evaluate(shape, guess * -1);

            switch (gjk_status)
            {

            case GJK::eStatus::Inside:
            {
                EPA epa;
                EPA::eStatus::_ epa_status = epa.Evaluate(gjk, guess * -1);
                if (epa_status != EPA::eStatus::Failed)
                {
                    Vector3 w0 = Vector3(0, 0, 0);
                    for (U i = 0; i < epa.m_result.rank; ++i)
                    {
                        // w0 是 物体1在物体2的最深穿透点在世界坐标下的坐标
                        // Support 返回的是全局坐标系的点，所以可以判断w0是全局坐标系下的点。
                        w0 = w0 + shape.Support(epa.m_result.c[i]->d, 1) * epa.m_result.p[i];

                    }
                    Matrix4x4 wtrs1 = Shape1->m_worldTransform.inverse();
                    result.status = sResults::Penetrating;
                    result.witnessesInFirstLocal[0] = wtrs1 * w0;								// 物体1在物体2中的最深穿透点在物体1下的坐标
                    Vector3 secondObjectPointInFirstObject = w0 - epa.m_normal * epa.m_depth;
                    result.witnessesInFirstLocal[1] = wtrs1 * secondObjectPointInFirstObject;	// 物体2在物体1中的最深穿透点在物体1下的坐标
                    result.witnessInGlobal[0] = w0;
                    result.witnessInGlobal[1] = secondObjectPointInFirstObject;
                    result.normal = epa.m_normal;	// 全局坐标下，由物体1指向物体2
                    result.distance = epa.m_depth;	// 距离为正数
                    return true;
                }
                else
                {
                    result.status = sResults::EPA_Failed;
                }

            }
            case GJK::eStatus::Failed:
            {
                result.status = sResults::GJK_Failed;
            }
            default:
            {
                break;
            }

            }
            return false;
        }

        void CollisionDetector::GenerateTangents(ContactPoint& contactPoint)
        {
            if (contactPoint.normal.m_x >= 0.57735f)
            {
                contactPoint.tangent1.m_x = contactPoint.normal.m_y;
                contactPoint.tangent1.m_y = -contactPoint.normal.m_x;
                contactPoint.tangent1.m_z = 0;
            }
            else
            {
                contactPoint.tangent1.m_x = 0;
                contactPoint.tangent1.m_y = contactPoint.normal.m_z;
                contactPoint.tangent1.m_z = -contactPoint.normal.m_y;
            }
            contactPoint.tangent1.normalize();
            contactPoint.tangent2 = contactPoint.normal.crossProduct( contactPoint.tangent1);
        }

        void Jacobian::Init(std::shared_ptr<ContactManifold> manifold, int idx, JacobianType jt, Vector3 dir, float dt)
        {
            jacobinType = jt;

            m_jva = dir * -1;
            m_jwa = manifold->contactPoints[idx].rA.crossProduct(dir) * -1;
            m_jvb = dir;
            m_jwb = manifold->contactPoints[idx].rB.crossProduct(dir);

            m_bias = 0.0f;

            if (jacobinType == JacobianType::Normal)
            {
                float betaA = manifold->colliderA->GetContactBeta();
                float betaB = manifold->colliderB->GetContactBeta();
                float beta = betaA * betaB;

                float restitutionA = manifold->colliderA->GetRestitution();
                float restitutionB = manifold->colliderB->GetRestitution();
                float restitution = restitutionA * restitutionB;

                Vector3 va = manifold->colliderA->GetVelocity();
                Vector3 wa = manifold->colliderA->GetAngularVelocity();
                Vector3 ra = manifold->contactPoints[idx].rA;

                Vector3 vb = manifold->colliderB->GetVelocity();
                Vector3 wb = manifold->colliderB->GetAngularVelocity();
                Vector3 rb = manifold->contactPoints[idx].rB;

                m_bias = 0;
                if (jacobinType == JacobianType::Normal)
                {
                    // See
                    // http://allenchou.net/2013/12/game-physics-resolution-contact-constraints/
                    Vector3 relativeVelocity = vb + wb.crossProduct(rb) - va - wa.crossProduct(ra);
                    float closingVelocity = relativeVelocity.dotProduct(dir);
                    float SlopP = 0.0005, SlopR = 0.5;//stablization terms
                    m_bias = -(beta / dt) * fmaxf(manifold->contactPoints[idx].penetrationDistance - SlopP, 0) + restitution * fmaxf(closingVelocity - SlopR, 0);
                }
            }

            // http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/
            // https://www.youtube.com/watch?v=pmdYzNF9x34
            // effectiveMass
            
            auto rigidA = manifold->colliderA;
            auto rigidB = manifold->colliderB;

            Vector3 rva = rigidA->GetInverInertiaLocal() * m_jwa;
            Vector3 rvb = rigidB->GetInverInertiaLocal() * m_jwb;

            float k =
                rigidA->GetInverseMass() + rigidB->GetInverseMass()
                + m_jwa.dotProduct(rva)
                + m_jwb.dotProduct(rvb);

            m_effectiveMass = 1 / k;
            m_totalLambda = 0;

        }

        void Jacobian::Solve(std::shared_ptr<ContactManifold> manifold, int idx, Vector3 dir, float dt)
        {
            ContactPoint& point = manifold->contactPoints[idx];

            float jv = m_jva.dotProduct(manifold->colliderA->GetVelocity())
                + m_jwa.dotProduct(manifold->colliderA->GetAngularVelocity())
                + m_jvb.dotProduct(manifold->colliderB->GetVelocity())
                + m_jwb.dotProduct(manifold->colliderB->GetAngularVelocity());

            float lambda = m_effectiveMass * (-(jv + m_bias));
            float oldTotalLambda = m_totalLambda;
            switch (jacobinType)
            {
            case JacobianType::Normal:
                m_totalLambda = std::max(m_totalLambda + lambda, 0.0f);
                break;

            case JacobianType::Tangent:
                float friction = manifold->colliderA->GetFriction() * manifold->colliderB->GetFriction();
                float maxFriction = friction * point.m_jN.m_totalLambda;
                m_totalLambda = Clamp(m_totalLambda + lambda, -maxFriction, maxFriction);
                break;
            }
            lambda = m_totalLambda - oldTotalLambda;

            if (manifold->colliderA->IsDynamic)
            {
                Vector3 va = manifold->colliderA->GetVelocity();
                Vector3 vadelta = m_jva * manifold->colliderA->GetInverseMass() * lambda;
                manifold->colliderA->SetVelocity(va + vadelta);

                Vector3 wa = manifold->colliderA->GetAngularVelocity();
                Vector3 wadelta = (manifold->colliderA->GetInverseInertiaTensorWorld() * m_jwa) * lambda;
                manifold->colliderA->SetAngularVelocity(wa + wadelta);
            }

            if (manifold->colliderB->IsDynamic)
            {
                Vector3 vb = manifold->colliderB->GetVelocity();
                Vector3 vbdelta = m_jvb * manifold->colliderB->GetInverseMass() * lambda;
                manifold->colliderB->SetVelocity(vb + vbdelta);

                Vector3 wb = manifold->colliderB->GetAngularVelocity();
                Vector3 wbdelta = (manifold->colliderB->GetInverseInertiaTensorWorld() * m_jwb) * lambda;
                manifold->colliderB->SetAngularVelocity(wb + wbdelta);

            }
        }

        void ContactManifold::AddContact(ContactPoint point)
        {
            if (contactPointCount >= 4)
            {
                return;
            }

            contactPointCount += 1;
            contactPoints[contactPointCount] = point;
        }

        void ContactManifold::UpdateContacts()
        {

        }

}
}
