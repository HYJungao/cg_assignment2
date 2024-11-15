
#include "AreaLight.hpp"


namespace FW {


void AreaLight::draw(const Mat4f& worldToCamera, const Mat4f& projection) {
    glUseProgram(0);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf((float*)&projection);
    glMatrixMode(GL_MODELVIEW);
    Mat4f S = Mat4f::scale(Vec3f(m_size,1));
    Mat4f M = worldToCamera *m_xform * S;
    glLoadMatrixf((float*)&M);
    glBegin(GL_TRIANGLES);
    glColor3fv( &m_E.x );
    glVertex3f(1,1,0); glVertex3f(1,-1,0); glVertex3f( -1,-1,0 );
    glVertex3f(1,1,0); glVertex3f( -1,-1,0 ); glVertex3f(-1,1,0); 
    glEnd();
}

void AreaLight::sample(float& pdf, Vec3f& p, int base, Random& rnd) {
    // YOUR CODE HERE (R2):
    // You should draw a random point on the light source and evaluate the PDF.
    // Store the results in "pdf" and "p".
    // 
    // Note: The "size" member is _one half_ the diagonal of the light source.
    // That is, when you map the square [-1,1]^2 through the scaling matrix
    // 
    // S = ( size.x    0   )
    //     (   0    size.y )
    // 
    // the result is the desired light source quad (see draw() function above).
    // This means the total area of the light source is 4*size.x*size.y.
    // This has implications for the computation of the PDF.

    // For extra credit, implement QMC sampling using some suitable sequence.
    // Use the "base" input for controlling the progression of the sequence from
    // the outside. If you only implement purely random sampling, "base" is not required.

    // (this does not do what it's supposed to!)
    Vec2f tmp = rnd.getVec2f(-1, 1);

    p = m_xform * Vec3f(tmp.x * m_size.x, tmp.y * m_size.y, 0);

    pdf = 1.0f / (4 * m_size.x * m_size.y);
}

float AreaLight::halton(int index, int base) {
    float result = 0;
    float f = 1.0 / base;
    int i = index;
    while (i > 0) {
        result = result + f * (i % base);
        i = i / base;
        f = f / base;
    }
    return result;
}

void AreaLight::sampleHalton(float& pdf, Vec3f& p, int base1, int base2, int index)
{
    float a = halton(index, base1) * 2 - 1;
    float b = halton(index, base2) * 2 - 1;

    p = m_xform * Vec3f(a * m_size.x, b * m_size.y, 0);

    pdf = 1.0f / (4 * m_size.x * m_size.y);
}

void AreaLight::StratifiedSample(float& pdf, Vec3f& p, Random& rnd, float dx, float dy, float i, float j)
{
    float aa = -1 + i * dx;
    float ab = -1 + (i + 1) * dx;

    float ba = -1 + j * dy;
    float bb = -1 + (j + 1) * dy;
    float x = rnd.getF32(aa, ab);
    float y = rnd.getF32(ba, bb);

    p = m_xform * Vec3f(x * m_size.x, y * m_size.y, 0);

    pdf = 1.0f / (4 * m_size.x * m_size.y);
}

} // namespace FW
