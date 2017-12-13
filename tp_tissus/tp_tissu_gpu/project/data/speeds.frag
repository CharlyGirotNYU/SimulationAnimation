#version 120

uniform sampler2D positions;
uniform sampler2D speeds;
uniform sampler2D normals;

const float SIZE = 128.0;
#define SQRT2     (1.4142136)

const float DELTA  = (1.0 / SIZE);
#define GRID      (4.0 / SIZE)     //cloth has size 4x4

#define DT 0.0010
#define damping 50
#define K 8000
#define Kw 0.01

const float GRAVITY = 9.81;


void main (void)
{

    float u = gl_TexCoord[0].s;
    float v = gl_TexCoord[0].t;
    vec2 uv = gl_TexCoord[0].st;

    vec3 speed  = texture2D (speeds   ,uv).xyz;
    vec3 pos    = texture2D (positions,uv).xyz;
    vec3 normal = texture2D (normals  ,uv).xyz;

    speed = (1-damping*DT)*speed;
    speed.y -= DT*GRAVITY;


    vec2 structural[4];
    structural[0] = vec2(1,0);
    structural[1] = vec2(-1,0);
    structural[2] = vec2(0,1);
    structural[3] = vec2(0,-1);

    vec2 shearing[4];
    shearing[0] = vec2(1,1);
    shearing[1] = vec2(1,-1);
    shearing[2] = vec2(-1,-1);
    shearing[3] = vec2(-1,1);

    vec2 bending[4];
    bending[0] = vec2(2,0);
    bending[1] = vec2(-2,0);
    bending[2] = vec2(0,2);
    bending[3] = vec2(0,-2);

    //verifie que les positions dans dans les limites attendues
    //structural spring
    bool ok_structural[4];
    ok_structural[0] = u<(1-DELTA);  ok_structural[1] = u>DELTA;
    ok_structural[2] = v<(1-DELTA);  ok_structural[3] = v>DELTA;

    bool ok_shearing[4];
    ok_shearing[0] = (u<(1-DELTA) && v<(1-DELTA));  ok_shearing[1] = (u<(1-DELTA) && v>DELTA);
    ok_shearing[2] = (u>DELTA && v>DELTA);  ok_shearing[3] = (u>DELTA &&  v<(1-DELTA));

    bool ok_bending[4];
    ok_bending[0] = u<(1-2*DELTA);  ok_bending[1] = u>2*DELTA;
    ok_bending[2] = v<(1-2*DELTA);  ok_bending[3] = v>2*DELTA;
    for(int k=0;k<4;++k)
    {
        if( ok_structural[k] )
        {
            //calculer ressorts et ajouter les forces à la vitesse
            vec3 vecteur_ressort = texture2D(positions,uv+structural[k]*DELTA).xyz-pos;
            float L = length(vecteur_ressort);
            float L_rest = 1.0f*GRID;
            speed += DT * K  * (L-L_rest) * vecteur_ressort/L;

        }
        //shearing spring
        if( ok_shearing[k] )
        {
            //calculer ressorts et ajouter les forces à la vitesse
            vec3 vecteur_ressort = texture2D(positions,uv+shearing[k]*DELTA).xyz-pos;
            float L = length(vecteur_ressort);
            float L_rest = SQRT2*GRID;
            speed += DT * K  * (L-L_rest) * vecteur_ressort/L;

        }

        //bending spring

        if( ok_bending[k] )
        {
            vec3 vecteur_ressort = texture2D(positions,uv+bending[k]*DELTA).xyz-pos;
            //calculer ressorts et ajouter les forces à la vitesse
            float L = length(vecteur_ressort);
            float L_rest = 2.0f*GRID;
            speed += DT * K  * (L-L_rest) * vecteur_ressort/L;

        }

        //Wind //Attention a la direction car on ne gere pas la collision du tissu avec lui même
        vec3 direction = vec3(-1.0,-1.0,0.0);
        speed += Kw * dot(direction,normal) * normal/length(normal);

    }


    gl_FragColor = vec4 (speed, 0.0);
}
