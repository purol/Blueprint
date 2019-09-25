# include <SDL.h>
# include <SDL_image.h>
# include <SDL_opengl.h>
# include <stdio.h>
#include <limits.h> 
# include <float.h>
# include <stdlib.h>
# include <math.h>
# include <SDL_ttf.h>
# include <SDL_mixer.h>
# include <time.h>

#pragma comment (lib,"SDL2")
#pragma comment (lib,"SDL2main")
#pragma comment (lib,"SDL2_image")
#pragma comment (lib, "opengl32.lib")
#pragma comment (lib,"SDL2_ttf")
#pragma comment (lib,"SDL2_mixer")
#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")

# define Length (40.0f/50.0f) // lenght of one block
# define fpss 130.0
# define g 9.8f
# define pi 3.1415926535f
# define shocknum 5
# define warmth 0.01f
# define beta 0.1f
# define MAX_VERTEX 11

# define threshold 0.0002f
# define slop 0.01f

# define trans 50.0f

# define bomb_angle_divide 32
# define bomb_length_divide ((40.0f/50.0f)/2.0f)
# define bomb_lenght_divide_num 6
# define bomb_animation_time 0.025f

# define sound_threshold_speed (50.0f/50.0f)
# define sound_max_speed (800.0f/50.0f)

struct material {
	float restitution;
	float density;
	float us; // u static constant
	float uk; // u kinetic constant
	char number; // identify number of material
}; typedef struct material Material;

struct shape {
	int vertex_num; // number of vertex on shape
	float ** vertex; // vertex vector from center of mass
	int * vertex_contact_index;
	bool * vertex_contact;
	float ** normal; // normal vector of polygon
	float volume;
	float inertia_constant; // I/(m*V) value
	int number; // identify number of shape
	int layer; // default value = -1. if objects have same positive value, then they don't collision
	float max_distance;

}; typedef struct shape Shape;

struct bomb {
	bool explosion[bomb_angle_divide];
	float x[2];
	float momentum;
}; typedef struct bomb Bomb;

struct object {
	Shape * shape; // shape of object

	float x[2]; // position vector x of COM
	float v[2]; // velocity vector v of COM
	float a[2]; // acceleration vector a of COM

	float th; // angle value theta to COM
	float w; // angular velocity w to COM
	float alp; // angular acceleration alpha to COM

	float F[2]; // force on object of COM

	float inv_mass; // inverse of mass
	float inv_inertia; // inverse of inertial

	float delw;
	float delv[2];

	float past_delw;
	float past_delv[2];

	bool stable;

	Material material;

}; typedef struct object Object;

struct contact_information {
	bool run;
	int numA;
	int numB;
	int indexA;
	int indexB;
	float rA[2]; // position vector from center of Mass A
	float rB[2]; // position vector from center of Mass B
	float normal_impulse;
	float tangent_impulse;
	float normal_mass;
	float tangent_mass;
	float velocity_bias;
	float depth;
}; typedef struct contact_information cinf;

struct save {
	int save_stage; // if save_stage value is 0 then it means stage 1
	int save_step;
	float record[6][8][2]; // [stage][step][height/budget]
	int version;
}; typedef struct save Save;

/*
struct tempt_save {
	int save_stage; // if save_stage value is 0 then it means stage 1
	int save_step;
	float record[5][8][2]; // [stage][step][height/budget]
	int version;
}; typedef struct tempt_save TSave;
*/

void cycle1(Object A[], int n, int NumberOfObject, cinf contact[]);
void one_term(Object A[], int NumberOfObject, cinf contact[]);
void set_shape(Shape * shape, int n);
void force_zero(Object * A);
void find_impulse(Object * A, Object * B, int NumberOfObject, cinf contact[], int numA, int numB);
void gravity(Object * A);
void movement(Object * A);
void apply_force(Object *A);
float clamp(float a, float min, float max);
void positionalcorrection(Object * A, Object * B);
void set_material(Object * object, int n);
void set_object(Object * A, int shape, int material, float x1, float x2, float v1, float v2, float a1, float a2, float th, float w, float alp, float F1, float F2);
void set_mass(Object * object);
void set_bomb(Bomb * b, float x1, float x2, float momentum);

void v_sum(float v1[2], float v2[2], float v_out[2]);

void v_sub(float v1[2], float v2[2], float v_out[2]);

float v_dot(float v1[2], float v2[2]);

float v_cross(float v1[2], float v2[2]);

void v_cross_k(float vec[2], float k, float v_out[2]);

void v_cross_k_inv(float k, float vec[2], float v_out[2]);

float v_magnitude(float v[2]);

void v_normalization(float v_in[2], float v_out[2]);

void v_normalization_s(float v_in[2]);

void mat_v_product(float mat[2][2], float v[2], float v_out[2]);

void set_rot_mat(float mat[2][2], float *radius);

void find_normal_v(float v[2], float dir[2], float v_out[2]);

void find_perpend_v(float v[2], float dir[2], float v_out[2]);

void v_mult(float v[2], float n, float v_out[2]);

float minimum(float a, float b);

float maximum(float a, float b);

float absolute(float a);

void print_digit(int budget, int * hundreds, int * tenth, int * units);

void draw(Object A[], int NumberOfObject);

float click_perception(float mouse_x, float mouse_y, Object * A);

void reassign_vertex(Object * A);

void delta_zero(Object * A);

void stability_zero(Object * A);

void collision_check(Object * A, Object * B, int NumberOfObject, cinf contact[], int numA, int numB);

void uncollide_check(Object A[], int NumberOfObject, cinf contact[]);

void warmstarting(Object A[], int NumberOfObject, cinf contact[]);

void one_term_with_earthquake(Object A[], int NumberOfObject, cinf contact[], int earthquake_type, float *time, float amp, float frequency);

void earthquake(Object * A, float time, int type, float amp, float frequency);

void map_editor(Object **A, int * NumberOfObject, int * budget, float * height, float * starttime, float * timeduration, float * endtime, int * earthquake_type, float *amp, float *frequency, int stage, int step, bool * explo, int * bomb_population, Bomb ** bomb);

void bomb_function(Object A[], int NumberOfObject, Bomb b[], int NumberOfBomb);

void cat_hurt_check(Object object[], int population, cinf contact[], bool * cat_hurt, float cat_hurt_coordinate[]);


SDL_Window * window;
SDL_Renderer * renderer;

SDL_Texture *texture[4];


SDL_Texture * loadTexture(const char *file) {
	SDL_Surface * surface;
	SDL_Texture *texture;

	surface = IMG_Load(file);
	if (surface == NULL) {
		printf("%s파일을 읽을 수 없습니다.\n", file);
		return NULL;
	}
	texture = SDL_CreateTextureFromSurface(renderer, surface);
	if (texture == NULL) {
		printf("텍스쳐를 생성할 수 없습니다.\n");
	}
	SDL_FreeSurface(surface);
	return texture;
}

void stretchTextureEx(SDL_Renderer *renderer, float x, float y, float w, float h, SDL_Texture *texture, float angle, SDL_RendererFlip flip = SDL_FLIP_NONE) {
	SDL_Rect src, dst;
	SDL_Point center;

	src.x = src.y = 0;
	SDL_QueryTexture(texture, NULL, NULL, &src.w, &src.h);

	dst.x = (int)x;
	dst.y = (int)y;
	dst.w = (int)w;
	dst.h = (int)h;

	center.x = (int)(w / 2.0);
	center.y = (int)(h / 2.0);

	SDL_RenderCopyEx(renderer, texture, &src, &dst, angle, &center, flip);
}

void stretchTextureEx_revise(SDL_Renderer *renderer, float x, float y, float w, float h, float CX, float CY, SDL_Texture *texture, float angle, SDL_RendererFlip flip = SDL_FLIP_NONE) {
	SDL_Rect src, dst;
	SDL_Point center;

	src.x = src.y = 0;
	SDL_QueryTexture(texture, NULL, NULL, &src.w, &src.h);

	dst.x = (int)x;
	dst.y = (int)y;
	dst.w = (int)w;
	dst.h = (int)h;

	center.x = (int)CX;
	center.y = (int)CY;

	SDL_RenderCopyEx(renderer, texture, &src, &dst, angle, &center, flip);
}

void drawTexture(SDL_Renderer *renderer, float x, float y, SDL_Texture *texture) {
	SDL_Rect src, dst;

	src.x = src.y = 0;
	SDL_QueryTexture(texture, NULL, NULL, &src.w, &src.h);
	
	dst.x = (int)x;
	dst.y = (int)y;
	dst.w = src.w;
	dst.h = src.h;

	SDL_RenderCopy(renderer, texture, &src, &dst);
}

static const char *arrow_cursor[] = {
	/* width height num_colors chars_per_pixel */
	"    32    32        3            1",
	/* colors */
	"X c #000000",
	". c #ffffff",
	"  c None",
	/* pixels */
	"X                               ",
	"XX                              ",
	"X.X                             ",
	"X..X                            ",
	"X...X                           ",
	"X....X                          ",
	"X.....X                         ",
	"X......X                        ",
	"X.......X                       ",
	"X........X                      ",
	"X.........X                     ",
	"X..........X                    ",
	"X...........X                   ",
	"X..........X                    ",
	"X.........X                     ",
	"X........X                      ",
	"X.......X                       ",
	"XXXXXXXX                        ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"0,0"
};

static SDL_Cursor *init_system_cursor(const char *image[])
{
	int i, row, col;
	Uint8 data[4 * 32];
	Uint8 mask[4 * 32];
	int hot_x, hot_y;

	i = -1;
	for (row = 0; row<32; ++row) {
		for (col = 0; col<32; ++col) {
			if (col % 8) {
				data[i] <<= 1;
				mask[i] <<= 1;
			}
			else {
				++i;
				data[i] = mask[i] = 0;
			}
			switch (image[4 + row][col]) {
			case 'X':
				data[i] |= 0x01;
				mask[i] |= 0x01;
				break;
			case '.':
				mask[i] |= 0x01;
				break;
			case ' ':
				break;
			}
		}
	}
	sscanf(image[4 + row], "%d,%d", &hot_x, &hot_y);
	return SDL_CreateCursor(data, mask, 32, 32, hot_x, hot_y);
}

int main(int argc, char **argv) {
	int antialiasing; // 1, 2, 4, 8
	int graphic_quality; // -1, 0, 1
	float music_sound; // 0~128
	float chunk_sound; // 0~128
	FILE * config;
	config = fopen("config", "rb");
	if (config == NULL) {
		antialiasing = 2;
		graphic_quality = 0;
		music_sound = 64;
		chunk_sound = 64;
		config = fopen("config", "wb");
		fprintf(config, "%d %d %f %f", antialiasing, graphic_quality, music_sound, chunk_sound);
		fclose(config);
	}
	else {
		fscanf(config, "%d %d %f %f", &antialiasing, &graphic_quality, &music_sound, &chunk_sound);
		fclose(config);
	}

	SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO);
	TTF_Init();
	Mix_Init(MIX_INIT_OGG);
	if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0) {
		printf("SDL_mixer initialization error : %s\n", Mix_GetError());
	}

	if (antialiasing == 1) {
		SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
	}
	else if (antialiasing == 2) {
		SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
		SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 2);
		SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "2");
	}
	else if (antialiasing == 4) {
		SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
		SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
		SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "2");
	}
	else if (antialiasing == 8) {
		SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
		SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 8);
		SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "2");
	}

	Save save;
	FILE * savefile;
	savefile = fopen("save.sav", "rb");

	if (savefile == NULL) {
		int i, j;
		save.save_stage = 0;
		save.save_step = 0;
		for (i = 0; i < 6; i++) for (j = 0; j < 8; j++) {
			save.record[i][j][0] = -1;
			save.record[i][j][1] = FLT_MAX;
		}
		save.version = 0;
		savefile = fopen("save.sav", "wb");
		fwrite(&save, sizeof(save), 1, savefile);
		fclose(savefile);
	}
	else {
		fread(&save, sizeof(save), 1, savefile);
		fclose(savefile);
	}
	/*
	TSave tsave;
	savefile = fopen("save.bin", "rb");
	fread(&tsave, sizeof(tsave), 1, savefile);
	fclose(savefile);
	save.save_stage = tsave.save_stage;
	save.save_step = tsave.save_step;
	int i, j;
	for (i = 0; i < 5; i++) for (j = 0; j < 8; j++) {
		save.record[i][j][0] = tsave.record[i][j][0];
		save.record[i][j][1] = tsave.record[i][j][1];
	}
	for (j = 0; j < 8; j++) {
		save.record[5][j][0] = -1;
		save.record[5][j][1] = FLT_MAX;
	}
	save.version = 0;
	savefile = fopen("save.bin", "wb");
	fwrite(&save, sizeof(save), 1, savefile);
	fclose(savefile);
	*/

	window = SDL_CreateWindow("Blueprint", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1360, 768, SDL_RENDERER_ACCELERATED);
	renderer = SDL_CreateRenderer(window, SDL_VIDEO_RENDER_OGL, 0);


	SDL_Cursor * basic_cursor;
	basic_cursor = init_system_cursor(arrow_cursor);
	SDL_SetCursor(basic_cursor);
	//	SDL_ShowCursor(0);

	//	SDL_Surface *icon;
	//	icon = IMG_Load("resource\\wood_one.png");
	//	SDL_SetWindowIcon(window, icon);

	bool quit = false;
	bool main_texture = false;
	bool option_texture = false;
	bool credits_texture = false;
	bool stage_texture = false;
	bool map_editor_texture = false;
	bool running_texture = false;
	bool startbutton = false; bool quitbutton = false; bool optionbutton = false; bool creditbutton = false;
	bool backbutton = false; bool nextbutton = false; bool mainmenubutton = false;
	bool BGM_button = false; bool Effect_button = false;
	int sound_click; int sound_button_click_x;
	bool step[8] = { false,false,false ,false ,false ,false ,false ,false };
	int stage_number = 0; int step_number = 0;
	float time_;
	int speed = 1;

	SDL_Event event;
	SDL_Texture * mainmenu;
	SDL_Texture * config_menu;
	SDL_Texture * arrow;

	SDL_Texture * off;
	SDL_Texture * two;
	SDL_Texture * four;
	SDL_Texture * eight;
	SDL_Texture * sixtyfps;
	SDL_Texture * onehundredtwentyfps;
	SDL_Texture * thirtyfps;

	SDL_Texture * credits_menu;

	SDL_Texture * stage;
	SDL_Texture * back; // back button
	SDL_Texture * next; // next button
	SDL_Texture * mainmenu_button; // mainmenu button
	SDL_Texture * lock; // lock icon
	SDL_Texture * stage_menu; // stage title

	SDL_Texture * map_editor_basic;
	SDL_Texture * trash_bin;
	SDL_Texture * write;
	SDL_Texture * play;
	SDL_Texture * map_editor_menu;
	SDL_Texture * editor_back;
	SDL_Texture * save_map;
	SDL_Texture * back_to_map;
	SDL_Texture * line_drag;
	SDL_Texture * map_edit_earthquake;
	SDL_Texture * small_arrow;
	SDL_Texture * LR_img;
	SDL_Texture * UD_img;
	SDL_Texture * UDLR_img;
	SDL_Texture * rotation_img;
	SDL_Texture * circle_img;
	SDL_Texture * swing_img;
	SDL_Texture * input_bar;

	SDL_Texture * map;
	SDL_Texture * back_to_stage;
	SDL_Texture * description;
	SDL_Texture * start;
	SDL_Texture * right_menu;
	SDL_Texture * question_round;
	SDL_Texture * stone;
	SDL_Texture * large_soil;
	SDL_Texture * ground_cone_small;
	SDL_Texture * hill;
	SDL_Texture * wood_one_block;
	SDL_Texture * wood_two_block;
	SDL_Texture * wood_three_block;
	SDL_Texture * wood_hexagon_block;
	SDL_Texture * wood_polygon1_block;
	SDL_Texture * wood_trapezoid_block;
	SDL_Texture * pre_wood_one_block;
	SDL_Texture * pre_wood_two_block;
	SDL_Texture * pre_wood_three_block;
	SDL_Texture * pre_wood_hexagon_block;
	SDL_Texture * pre_wood_polygon1_block;
	SDL_Texture * pre_wood_trapezoid_block;
	SDL_Texture * metal_one_block;
	SDL_Texture * metal_two_block;
	SDL_Texture * metal_three_block;
	SDL_Texture * metal_hexagon_block;
	SDL_Texture * metal_polygon1_block;
	SDL_Texture * metal_trapezoid_block;
	SDL_Texture * pre_metal_one_block;
	SDL_Texture * pre_metal_two_block;
	SDL_Texture * pre_metal_three_block;
	SDL_Texture * pre_metal_hexagon_block;
	SDL_Texture * pre_metal_polygon1_block;
	SDL_Texture * pre_metal_trapezoid_block;
	SDL_Texture * rock_one_block;
	SDL_Texture * rock_two_block;
	SDL_Texture * rock_three_block;
	SDL_Texture * rock_hexagon_block;
	SDL_Texture * rock_polygon1_block;
	SDL_Texture * rock_trapezoid_block;
	SDL_Texture * pre_rock_one_block;
	SDL_Texture * pre_rock_two_block;
	SDL_Texture * pre_rock_three_block;
	SDL_Texture * pre_rock_hexagon_block;
	SDL_Texture * pre_rock_polygon1_block;
	SDL_Texture * pre_rock_trapezoid_block;
	SDL_Texture * explosion1;
	SDL_Texture * explosion2;
	SDL_Texture * explosion3;
	SDL_Texture * explosion4;
	SDL_Texture * explosion5;
	SDL_Texture * explosion6;
	SDL_Texture * explosion7;
	SDL_Texture * explosion8;
	SDL_Texture * explosion9;
	SDL_Texture * explosion10;
	SDL_Texture * explosion11;
	SDL_Texture * explosion12;
	SDL_Texture * explosion13;
	SDL_Texture * explosion14;
	SDL_Texture * explosion15;
	SDL_Texture * explosion16;
	SDL_Texture * explosion17;
	SDL_Texture * bombimg;
	SDL_Texture * cat;
	SDL_Texture * line;
	SDL_Texture * redline;
	SDL_Texture * velocity_arrow;
	SDL_Texture * star;
	SDL_Texture * blood_dot;
	SDL_Texture * stop_retry;
	SDL_Texture * pause;
	SDL_Texture * normal_speed;
	SDL_Texture * x2speed;
	SDL_Texture * speed_select;

	SDL_Texture * five$;
	SDL_Texture * seven$;
	SDL_Texture * ten$;
	SDL_Texture * thirteen$;
	SDL_Texture * fifteen$;
	SDL_Texture * eighteen$;
	SDL_Texture * twenty$;
	SDL_Texture * twentythree$;
	SDL_Texture * twentysix$;
	SDL_Texture * twentyseven$;
	SDL_Texture * thirty$;
	SDL_Texture * thirtysix$;

	SDL_Texture * running_end_menu;
	SDL_Texture * back_to_home;
	SDL_Texture * next_arrow;
	SDL_Texture * re_arrow;

	SDL_Texture * sound_bar;
	SDL_Texture * sound_button;

	TTF_Font *font25;
	TTF_Font *font30;
	TTF_Font *font32;
	TTF_Font *font40;
	TTF_Font *font70;
	SDL_Surface * text_surface;
	SDL_Texture *text;
	font25 = TTF_OpenFont("resource\\Archistico_Simple.ttf", 25);
	font30 = TTF_OpenFont("resource\\Archistico_Simple.ttf", 30);
	font32 = TTF_OpenFont("resource\\Archistico_Simple.ttf", 32);
	font40 = TTF_OpenFont("resource\\Archistico_Simple.ttf", 40);
	font70 = TTF_OpenFont("resource\\Archistico_Simple.ttf", 70);
	SDL_Color white = { 255,255,255,255 };
	char budget_string[100];

	SDL_Texture * reddot;
	reddot = loadTexture("resource\\reddot.png");

	Object * pre_object;
	bool retry = false;
	int pre_population;
	int pre_budget;

	bool next_click = false;

	enum Scene { main_menu = 1, stage_select1, stage_select2, stage_select3, stage_select4, stage_select5, stage_select6, map_edit, in_stage, option, credits };
	Scene scene = main_menu;

	float rendering_factor;
	if (graphic_quality == 1) rendering_factor = 1.0;
	else if (graphic_quality == 0) rendering_factor = 2.0;
	else if (graphic_quality == -1) rendering_factor = 4.0;

	int total_frame_start;
	int total_frame_end;
	float total_time;
	int total_delay_time;

	bool stage_to_select_menu = false;
	bool escape_map_editor = false;

	Mix_Music * main_music = NULL;
	Mix_Music * in_stage_music = NULL;
	Mix_Chunk * collide_BB = NULL;
	Mix_Chunk * collide_BE = NULL;
	Mix_Chunk * hammering_wood = NULL;
	Mix_Chunk * hammering_iron = NULL;
	Mix_Chunk * hammering_brick = NULL;
	Mix_Chunk * erase = NULL;
	Mix_Chunk * gear = NULL;
	Mix_Chunk * explosion = NULL;
	Mix_Chunk * page = NULL;
	Mix_Chunk * fanfare = NULL;
	Mix_Chunk * click_sound = NULL;
	Mix_Chunk * earthquake_sound = NULL;

	main_music = Mix_LoadMUS("sound\\main.ogg");
	in_stage_music = Mix_LoadMUS("sound\\in_stage.ogg");
	collide_BB = Mix_LoadWAV("sound\\collide_BB.wav");
	collide_BE = Mix_LoadWAV("sound\\collide_BE.wav");
	hammering_wood = Mix_LoadWAV("sound\\hammering_wood.wav");
	hammering_iron = Mix_LoadWAV("sound\\hammering_iron.wav");
	hammering_brick = Mix_LoadWAV("sound\\hammering_brick.wav");
	erase = Mix_LoadWAV("sound\\erase.wav");
	gear = Mix_LoadWAV("sound\\gear.wav");
	explosion = Mix_LoadWAV("sound\\explosion.wav");
	page = Mix_LoadWAV("sound\\page.wav");
	fanfare = Mix_LoadWAV("sound\\fanfare.wav");
	click_sound = Mix_LoadWAV("sound\\click.wav");
	earthquake_sound = Mix_LoadWAV("sound\\earthquake_sound.wav");
	if (main_music == NULL) printf("fail to load music : %s\n", Mix_GetError());

	Mix_VolumeMusic((int)music_sound);
	Mix_PlayMusic(main_music, -1);

	Mix_VolumeChunk(hammering_wood, (int)chunk_sound);
	Mix_VolumeChunk(hammering_iron, (int)chunk_sound);
	Mix_VolumeChunk(hammering_brick, (int)chunk_sound);
	Mix_VolumeChunk(erase, (int)chunk_sound);
	Mix_VolumeChunk(gear, (int)chunk_sound);
	Mix_VolumeChunk(explosion, (int)chunk_sound);
	Mix_VolumeChunk(page, (int)chunk_sound);
	Mix_VolumeChunk(fanfare, (int)chunk_sound);
	Mix_VolumeChunk(click_sound, (int)chunk_sound);
	Mix_VolumeChunk(earthquake_sound, (int)chunk_sound);

	while (!quit) {
		if (stage_to_select_menu == true) stage_to_select_menu = false;
		if (escape_map_editor == true) escape_map_editor = false;
		total_frame_start = SDL_GetPerformanceCounter();

		if (scene == main_menu) {
			if (main_texture == false) {
				mainmenu = loadTexture("resource\\main_menu.png");
				arrow = loadTexture("resource\\arrow.png");
				main_texture = true;
			}

			stretchTextureEx(renderer, 0, 0, 1360, 768, mainmenu, 0);
			while (SDL_PollEvent(&event)) {
				switch (event.type) {
				case SDL_QUIT:
					quit = true;
					break;
				case SDL_MOUSEBUTTONDOWN:
					if (event.button.button == SDL_BUTTON_LEFT) {
						int x, y;
						x = event.motion.x;
						y = event.motion.y;
						if (x >= 1111 && x <= 1267 && y >= 368 && y <= 415) { // click start button
							scene = stage_select1;
							stage_number = 1;
							main_texture = false;
							SDL_DestroyTexture(mainmenu);
							SDL_DestroyTexture(arrow);
							Mix_PlayChannel(-1, click_sound, 0);
							startbutton = false;
						}
						else if (x >= 1067 && x <= 1267 && y >= 433 && y <= 498) { // click option button
							scene = option;
							main_texture = false;
							optionbutton = false;
							SDL_DestroyTexture(mainmenu);
							Mix_PlayChannel(-1, click_sound, 0);
//							SDL_DestroyTexture(arrow);
						}
						else if (x >= 1051 && x <= 1267 && y >= 511 && y <= 560) { // click credits button
							scene = credits;
							main_texture = false;
							creditbutton = false;
							SDL_DestroyTexture(mainmenu);
							Mix_PlayChannel(-1, click_sound, 0);
						}
						else if (x >= 1121 && x <= 1267 && y >= 585 && y <= 658) { // click quit button
							quit = true;
							main_texture = false;
							SDL_DestroyTexture(mainmenu);
							SDL_DestroyTexture(arrow);
							Mix_PlayChannel(-1, click_sound, 0);
						}
					}
					break;
				case SDL_MOUSEMOTION :
						int x, y;
						x = event.motion.x;
						y = event.motion.y;

						if (x >= 1111 && x <= 1267 && y >= 368 && y <= 415) { // on start button
							startbutton = true;
						}
						else if (x >= 1067 && x <= 1261 && y >= 433 && y <= 498) { // on option button
							optionbutton = true;
						}
						else if (x >= 1051 && x <= 1267 && y >= 511 && y <= 560) { // on credit button
							creditbutton = true;
						}
						else if (x >= 1121 && x <= 1267 && y >= 585 && y <= 658) { // on quit button
							quitbutton = true;
						}
						else {
							startbutton = false;
							quitbutton = false;
							optionbutton = false;
							creditbutton = false;
						}
					break;
				}
			}
			if (startbutton == true) { // on start button
				stretchTextureEx(renderer, 1046, 359, 65, 65, arrow, 0);
			}
			else if (optionbutton == true) { // on option button
				stretchTextureEx(renderer, 1002, 433, 65, 65, arrow, 0);
			}
			else if (creditbutton == true) { // on creditbutton
				stretchTextureEx(renderer, 986, 511, 65, 65, arrow, 0);
			}
			else if (quitbutton == true) { // on quit button
				stretchTextureEx(renderer, 1056, 589, 65, 65, arrow, 0);
			}
		}
		else if (scene == option) {
			if (option_texture == false) {
				config_menu = loadTexture("resource\\config_menu_revise.png");
				off=loadTexture("resource\\off.png");
				two = loadTexture("resource\\2.png");
				four = loadTexture("resource\\4.png");
				eight = loadTexture("resource\\8.png");
				sixtyfps = loadTexture("resource\\60fps.png");
				onehundredtwentyfps = loadTexture("resource\\120fps.png");
				thirtyfps = loadTexture("resource\\30fps.png");
				sound_bar= loadTexture("resource\\sound_bar.png");
				sound_button = loadTexture("resource\\sound_button.png");
				option_texture = true;
			}
			drawTexture(renderer, 0, 0, config_menu);
			if(antialiasing==1) drawTexture(renderer, 1163, 341, off);
			else if(antialiasing==2) drawTexture(renderer, 1177, 341, two);
			else if(antialiasing==4) drawTexture(renderer, 1177, 335, four);
			else if(antialiasing==8) drawTexture(renderer, 1177, 335, eight);

			if(graphic_quality==-1) drawTexture(renderer, 1092, 415, thirtyfps);
			else if (graphic_quality == 0) drawTexture(renderer, 1092, 415, sixtyfps);
			else if (graphic_quality == 1) drawTexture(renderer, 1087, 415, onehundredtwentyfps);

			if(antialiasing != 8) drawTexture(renderer, 1215, 330, arrow);
			if(antialiasing != 1)stretchTextureEx(renderer, 1109, 332, 65, 65, arrow, 180);
			if(graphic_quality!=1) drawTexture(renderer, 1215, 400, arrow);
			if (graphic_quality != -1) stretchTextureEx(renderer, 1022, 402, 65, 65, arrow, 180);

			drawTexture(renderer, 1085, 508, sound_bar); //BGM
			drawTexture(renderer, 1085+(int)(music_sound*1.515625f-9.5f), 487, sound_button);
			drawTexture(renderer, 1085, 590, sound_bar); //Effect
			drawTexture(renderer, 1085 + (int)(chunk_sound*1.515625f - 9.5f), 569, sound_button);

			while (SDL_PollEvent(&event)) {
				switch (event.type) {
				case SDL_QUIT:
					quit = true;
					break;
				case SDL_MOUSEBUTTONDOWN:
					if (event.button.button == SDL_BUTTON_LEFT) {
						int x, y;
						x = event.motion.x;
						y = event.motion.y;
						if (x >= 1215 && x <= 1280 && y >= 330 && y <= 395 && antialiasing != 8) { antialiasing = antialiasing * 2; Mix_PlayChannel(-1, click_sound, 0); }// click increase antialiasing arrow
						else if (x >= 1109 && x <= 1154 && y >= 332 && y <= 397 && antialiasing != 1) { antialiasing = antialiasing / 2; Mix_PlayChannel(-1, click_sound, 0); } // click decrease antialiasing arrow
						else if (x >= 1215 && x <= 1280 && y >= 400 && y <= 465 && graphic_quality != 1) { graphic_quality = graphic_quality + 1; Mix_PlayChannel(-1, click_sound, 0); } // click increase animation quality
						else if (x >= 1022 && x <= 1087 && y >= 402 && y <= 467 && graphic_quality != -1) { graphic_quality = graphic_quality - 1; Mix_PlayChannel(-1, click_sound, 0); } // click decrease animation quality
						else if (x>= 1085 + (int)(music_sound*1.515625f - 9.5f) && x<1104 + (int)(music_sound*1.515625f - 9.5f) && y>= 487 && y<529) {
							BGM_button = true;
							sound_click = x;
							sound_button_click_x = x - 1085 - (int)(music_sound*1.515625f);
							Mix_PlayChannel(-1, click_sound, 0);
						}
						else if (x >= 1085 + (int)(chunk_sound*1.515625f - 9.5f) && x<1104 + (int)(chunk_sound*1.515625f - 9.5f) && y >= 569 && y<611) {
							Effect_button = true;
							sound_click = x;
							sound_button_click_x = x - 1085 - (int)(chunk_sound*1.515625f);
							Mix_PlayChannel(-1, click_sound, 0);
						}
						else if (x >= 1121 && x <= 1267 && y >= 647 && y <= 695) {	// click back button
							scene = main_menu;
							SDL_DestroyTexture(config_menu);
							SDL_DestroyTexture(off);
							SDL_DestroyTexture(two);
							SDL_DestroyTexture(four);
							SDL_DestroyTexture(eight);
							SDL_DestroyTexture(sixtyfps);
							SDL_DestroyTexture(onehundredtwentyfps);
							SDL_DestroyTexture(thirtyfps);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(sound_bar);
							SDL_DestroyTexture(sound_button);
							backbutton = false;
							option_texture = false;
							config = fopen("config", "wb");
							fprintf(config, "%d %d %f %f", antialiasing, graphic_quality, music_sound, chunk_sound);
							fclose(config);
							Mix_PlayChannel(-1, click_sound, 0);
						}
					}
					break;
				case SDL_MOUSEMOTION:
					int x, y;
					x = event.motion.x;
					y = event.motion.y;

					if (x >= 1121 && x <= 1267 && y >= 647 && y <= 695) { // on back button
						backbutton = true;
					}
					else backbutton = false;
					if (BGM_button == true && music_sound>0 && music_sound<128) {
						music_sound= music_sound+(x - sound_click)*0.65979f;
						if (music_sound < 0)music_sound = 0;
						else if (music_sound > 128) music_sound = 128;
						sound_click = x;
						Mix_VolumeMusic((int)music_sound);
					}
					else if (BGM_button == true && music_sound == 0 && 1085 + sound_button_click_x <= x) {
						music_sound = music_sound + (x - 1085 - sound_button_click_x)*0.65979f;
						if (music_sound < 0)music_sound = 0;
						else if (music_sound > 128) music_sound = 128;
						sound_click = x;
						Mix_VolumeMusic((int)music_sound);
					}
					else if (BGM_button == true && music_sound == 128 && 1279 + sound_button_click_x >= x) {
						music_sound = music_sound + (x - 1279 - sound_button_click_x)*0.65979f;
						if (music_sound < 0)music_sound = 0;
						else if (music_sound > 128) music_sound = 128;
						sound_click = x;
						Mix_VolumeMusic((int)music_sound);
					}
					else if (Effect_button == true && chunk_sound>0 && chunk_sound<128) {
						chunk_sound = chunk_sound + (x - sound_click)*0.65979f;
						if (chunk_sound < 0)chunk_sound = 0;
						else if (chunk_sound > 128) chunk_sound = 128;
						sound_click = x;
						Mix_VolumeChunk(hammering_wood, (int)chunk_sound);
						Mix_VolumeChunk(hammering_iron, (int)chunk_sound);
						Mix_VolumeChunk(hammering_brick, (int)chunk_sound);
						Mix_VolumeChunk(erase, (int)chunk_sound);
						Mix_VolumeChunk(gear, (int)chunk_sound);
						Mix_VolumeChunk(explosion, (int)chunk_sound);
						Mix_VolumeChunk(page, (int)chunk_sound);
						Mix_VolumeChunk(fanfare, (int)chunk_sound);
						Mix_VolumeChunk(click_sound, (int)chunk_sound);
						Mix_VolumeChunk(earthquake_sound, (int)chunk_sound);
					}
					else if (Effect_button == true && chunk_sound == 0 && 1085 + sound_button_click_x <= x) {
						chunk_sound = chunk_sound + (x - 1085 - sound_button_click_x)*0.65979f;
						if (chunk_sound < 0)chunk_sound = 0;
						else if (chunk_sound > 128) chunk_sound = 128;
						sound_click = x;
						Mix_VolumeChunk(hammering_wood, (int)chunk_sound);
						Mix_VolumeChunk(hammering_iron, (int)chunk_sound);
						Mix_VolumeChunk(hammering_brick, (int)chunk_sound);
						Mix_VolumeChunk(erase, (int)chunk_sound);
						Mix_VolumeChunk(gear, (int)chunk_sound);
						Mix_VolumeChunk(explosion, (int)chunk_sound);
						Mix_VolumeChunk(page, (int)chunk_sound);
						Mix_VolumeChunk(fanfare, (int)chunk_sound);
						Mix_VolumeChunk(click_sound, (int)chunk_sound);
						Mix_VolumeChunk(earthquake_sound, (int)chunk_sound);
					}
					else if (Effect_button == true && chunk_sound == 128 && 1279 + sound_button_click_x >= x) {
						chunk_sound = chunk_sound + (x - 1279 - sound_button_click_x)*0.65979f;
						if (chunk_sound < 0)chunk_sound = 0;
						else if (chunk_sound > 128) chunk_sound = 128;
						sound_click = x;
						Mix_VolumeChunk(hammering_wood, (int)chunk_sound);
						Mix_VolumeChunk(hammering_iron, (int)chunk_sound);
						Mix_VolumeChunk(hammering_brick, (int)chunk_sound);
						Mix_VolumeChunk(erase, (int)chunk_sound);
						Mix_VolumeChunk(gear, (int)chunk_sound);
						Mix_VolumeChunk(explosion, (int)chunk_sound);
						Mix_VolumeChunk(page, (int)chunk_sound);
						Mix_VolumeChunk(fanfare, (int)chunk_sound);
						Mix_VolumeChunk(click_sound, (int)chunk_sound);
						Mix_VolumeChunk(earthquake_sound, (int)chunk_sound);
					}
					break;
				case SDL_MOUSEBUTTONUP :
					BGM_button = false;
					Effect_button = false;
					break;
				}
			}
			if (backbutton == true) { // on back button
				stretchTextureEx(renderer, 1056, 641, 65, 65, arrow, 0);
			}

		}
		else if (scene == credits) {
			if (credits_texture == false) {
				credits_menu = loadTexture("resource\\credits.png");
				credits_texture = true;
			}

			drawTexture(renderer, 0, 0, credits_menu);
			if(backbutton == true) stretchTextureEx(renderer, 1052, 647, 65, 65, arrow, 0);

			while (SDL_PollEvent(&event)) {
				switch (event.type) {
				case SDL_QUIT:
					quit = true;
					break;
				case SDL_MOUSEBUTTONDOWN:
					if (event.button.button == SDL_BUTTON_LEFT) {
						int x, y;
						x = event.motion.x;
						y = event.motion.y;
						if (x >= 1117 && x <= 1261 && y >= 653 && y <= 700) { // click back
							scene = main_menu;
							SDL_DestroyTexture(credits_menu);
							credits_texture = false;
							backbutton = false;
							Mix_PlayChannel(-1, click_sound, 0);
						}
					}
					break;
				case SDL_MOUSEMOTION:
					int x, y;
					x = event.motion.x;
					y = event.motion.y;
					if (x >= 1117 && x <= 1261 && y >= 653 && y <= 700) {
						backbutton = true;
					}
					else backbutton = false;
				}
			}
		}
		else if (scene == stage_select1|| scene == stage_select2|| scene == stage_select3|| scene == stage_select4|| scene == stage_select5 || scene == stage_select6) {

			if (stage_texture == false) {
				stage = loadTexture("resource\\stage_basic.png");
				arrow = loadTexture("resource\\arrow.png");
				back=loadTexture("resource\\back.png");
				next= loadTexture("resource\\next.png");
				mainmenu_button= loadTexture("resource\\mainmenu.png");
				lock = loadTexture("resource\\lock.png");
				if(stage_number==1) stage_menu= loadTexture("resource\\stage1menu.png");
				else if(stage_number==2) stage_menu = loadTexture("resource\\stage2menu.png");
				else if (stage_number == 3) stage_menu = loadTexture("resource\\stage3menu.png");
				else if (stage_number == 4) stage_menu = loadTexture("resource\\stage4menu.png");
				else if (stage_number == 5) stage_menu = loadTexture("resource\\stage5menu.png");
				else if (stage_number == 6) stage_menu = loadTexture("resource\\stage6menu.png");
				stage_texture = true;
			}

			stretchTextureEx(renderer, 0, 0, 1360, 768, stage, 0);
			drawTexture(renderer, 556, 56, stage_menu);
			drawTexture(renderer, 975, 669, mainmenu_button);
			if (scene != stage_select6) drawTexture(renderer, 1135, 57, next);
			drawTexture(renderer, 85, 57, back);

			if ((save.save_stage + 1) * 10 + (save.save_step + 1) < (int)(scene - 1) * 10 + 8)drawTexture(renderer, 1101, 439, lock);
			if ((save.save_stage + 1) * 10 + (save.save_step + 1) < (int)(scene - 1) * 10 + 7)drawTexture(renderer, 768, 439, lock);
			if ((save.save_stage + 1) * 10 + (save.save_step + 1) < (int)(scene - 1) * 10 + 6)drawTexture(renderer, 427, 439, lock);
			if ((save.save_stage + 1) * 10 + (save.save_step + 1) < (int)(scene - 1) * 10 + 5)drawTexture(renderer, 115, 433, lock);
			if ((save.save_stage + 1) * 10 + (save.save_step + 1) < (int)(scene - 1) * 10 + 4)drawTexture(renderer, 1101, 129, lock);
			if ((save.save_stage + 1) * 10 + (save.save_step + 1) < (int)(scene - 1) * 10 + 3)drawTexture(renderer, 768, 129, lock);
			if ((save.save_stage + 1) * 10 + (save.save_step + 1) < (int)(scene - 1) * 10 + 2)drawTexture(renderer, 427, 129, lock);
			if ((save.save_stage + 1) * 10 + (save.save_step + 1) < (int)(scene - 1) * 10 + 1)drawTexture(renderer, 115, 129, lock);

			while (SDL_PollEvent(&event)) {
				switch (event.type) {
				case SDL_QUIT:
					quit = true;
					break;
				case SDL_MOUSEBUTTONDOWN:
					if (event.button.button == SDL_BUTTON_LEFT) {
						int x, y;
						x = event.motion.x;
						y = event.motion.y;
						if (x >= 85 && x <= 246 && y >= 57 && y <= 117) { // click back
							if (scene == stage_select6) { scene = stage_select5; stage_number = 5; }
							else if (scene == stage_select5) { scene = stage_select4; stage_number = 4; }
							else if (scene == stage_select4) { scene = stage_select3; stage_number = 3; }
							else if (scene == stage_select3) { scene = stage_select2; stage_number = 2; }
							else if (scene == stage_select2) { scene = stage_select1; stage_number = 1; }
							else if (scene == stage_select1) {
								scene = map_edit;
								SDL_DestroyTexture(stage);
								SDL_DestroyTexture(arrow);
								SDL_DestroyTexture(back);
								SDL_DestroyTexture(next);
								SDL_DestroyTexture(mainmenu_button);
								SDL_DestroyTexture(lock);
								SDL_DestroyTexture(stage_menu);
								stage_texture = false;
								backbutton = false;
							}
							if (scene == stage_select1) stage_menu = loadTexture("resource\\stage1menu.png");
							else if (scene == stage_select2) stage_menu = loadTexture("resource\\stage2menu.png");
							else if (scene == stage_select3) stage_menu = loadTexture("resource\\stage3menu.png");
							else if (scene == stage_select4) stage_menu = loadTexture("resource\\stage4menu.png");
							else if (scene == stage_select5) stage_menu = loadTexture("resource\\stage5menu.png");
							else if (scene == stage_select6) stage_menu = loadTexture("resource\\stage6menu.png");
							Mix_PlayChannel(-1, page, 0);
						}
						else if (x >= 1135 && x <= 1293 && y >= 57 && y <= 119 && scene != stage_select6) { // click next
							if (scene == stage_select1) { scene = stage_select2; stage_number = 2; }
							else if (scene == stage_select2) { scene = stage_select3; stage_number = 3; }
							else if (scene == stage_select3) { scene = stage_select4; stage_number = 4; }
							else if (scene == stage_select4) { scene = stage_select5; stage_number = 5; }
							else if (scene == stage_select5) { scene = stage_select6; stage_number = 6; }
							if (scene == stage_select1) stage_menu = loadTexture("resource\\stage1menu.png");
							else if (scene == stage_select2) stage_menu = loadTexture("resource\\stage2menu.png");
							else if (scene == stage_select3) stage_menu = loadTexture("resource\\stage3menu.png");
							else if (scene == stage_select4) stage_menu = loadTexture("resource\\stage4menu.png");
							else if (scene == stage_select5) stage_menu = loadTexture("resource\\stage5menu.png");
							else if (scene == stage_select6) { stage_menu = loadTexture("resource\\stage6menu.png"); nextbutton = false; }
							Mix_PlayChannel(-1, page, 0);
						}
						else if (x >= 975 && x <= 1322 && y >= 669 && y <= 740) { // click mainmenu button
							stage_texture = false;
							mainmenubutton = false;
							scene = main_menu;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(lock);
							SDL_DestroyTexture(stage_menu);
							Mix_PlayChannel(-1, click_sound, 0);
						}
						else if (x >= 105 && x <= 211 && y >= 144 && y <= 261&&((save.save_stage + 1) * 10 + (save.save_step + 1) >= (int)(scene - 1) * 10 + 1)) { // click step1 button
							stage_texture = false;
							step[0] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(lock);
							SDL_DestroyTexture(stage_menu);
//							stage_number = scene - 1;
							step_number = 1;
							Mix_HaltMusic();
							Mix_FadeInMusic(in_stage_music, -1, 1000);
						}
						else if (x >= 383 && x <= 538 && y >= 141 && y <= 287 && ((save.save_stage + 1) * 10 + (save.save_step + 1) >= (int)(scene - 1) * 10 + 2)) { // click step2 button
							stage_texture = false;
							step[1] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(lock);
							SDL_DestroyTexture(stage_menu);
//							stage_number = scene - 1;
							step_number = 2;
							Mix_HaltMusic();
							Mix_FadeInMusic(in_stage_music, -1, 1000);
						}
						else if (x >= 699 && x <= 899 && y >= 141 && y <= 315 && ((save.save_stage + 1) * 10 + (save.save_step + 1) >= (int)(scene - 1) * 10 + 3)) { // click step3 button
							stage_texture = false;
							step[2] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(lock);
							SDL_DestroyTexture(stage_menu);
//							stage_number = scene - 1;
							step_number = 3;
							Mix_HaltMusic();
							Mix_FadeInMusic(in_stage_music, -1, 1000);
						}
						else if (x >= 1020 && x <= 1269 && y >= 141 && y <= 344 && ((save.save_stage + 1) * 10 + (save.save_step + 1) >= (int)(scene - 1) * 10 + 4)) { // click step4 button
							stage_texture = false;
							step[3] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(lock);
							SDL_DestroyTexture(stage_menu);
//							stage_number = scene - 1;
							step_number = 4;
							Mix_HaltMusic();
							Mix_FadeInMusic(in_stage_music, -1, 1000);
						}
						else if (x >= 106 && x <= 260 && y >= 414 && y <= 590 && ((save.save_stage + 1) * 10 + (save.save_step + 1) >= (int)(scene - 1) * 10 + 5)) { // click step5 button
							stage_texture = false;
							step[4] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(lock);
							SDL_DestroyTexture(stage_menu);
//							stage_number = scene - 1;
							step_number = 5;
							Mix_HaltMusic();
							Mix_FadeInMusic(in_stage_music, -1, 1000);
						}
						else if (x >= 370 && x <= 572 && y >= 414 && y <= 619 && ((save.save_stage + 1) * 10 + (save.save_step + 1) >= (int)(scene - 1) * 10 + 6)) { // click step6 button
							stage_texture = false;
							step[5] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(lock);
							SDL_DestroyTexture(stage_menu);
//							stage_number = scene - 1;
							step_number = 6;
							Mix_HaltMusic();
							Mix_FadeInMusic(in_stage_music, -1, 1000);
						}
						else if (x >= 687 && x <= 937 && y >= 445 && y <= 592 && ((save.save_stage + 1) * 10 + (save.save_step + 1) >= (int)(scene - 1) * 10 + 7)) { // click step7 button
							stage_texture = false;
							step[6] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(lock);
							SDL_DestroyTexture(stage_menu);
//							stage_number = scene - 1;
							step_number = 7;
							Mix_HaltMusic();
							Mix_FadeInMusic(in_stage_music, -1, 1000);
						}
						else if (x >= 1019 && x <= 1271 && y >= 414 && y <= 618 && ((save.save_stage + 1) * 10 + (save.save_step + 1) >= (int)(scene - 1) * 10 + 8)) { // click step8 button
							stage_texture = false;
							step[7] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(lock);
							SDL_DestroyTexture(stage_menu);
//							stage_number = scene - 1;
							step_number = 8;
							Mix_HaltMusic();
							Mix_FadeInMusic(in_stage_music, -1, 1000);
						}
					}
					break;
				case SDL_MOUSEMOTION:
					int x, y;
					x = event.motion.x;
					y = event.motion.y;

					if (x >= 85 && x <= 246 && y >= 57 && y <= 117) { // on back button
						backbutton = true;
					}
					else if (x >= 1135 && x <= 1293 && y >= 57 && y <= 119 && scene != stage_select6) { // on next button
						nextbutton = true;
					}
					else if (x >= 975 && x <= 1322 && y >= 669 && y <= 740) { // on mainmenu button
						mainmenubutton = true;
					}
					else if (x >= 105 && x <= 211 && y >= 144 && y <= 261) { // on step1 button
						step[0]= true;
					}
					else if (x >= 383 && x <= 538 && y >= 141 && y <= 287) { // on step2 button
						step[1] = true;
					}
					else if (x >= 699 && x <= 899 && y >= 141 && y <= 315) { // on step3 button
						step[2] = true;
					}
					else if (x >= 1020 && x <= 1269 && y >= 141 && y <= 344) { // on step4 button
						step[3] = true;
					}
					else if (x >= 106 && x <= 260 && y >= 414 && y <= 590) { // on step5 button
						step[4] = true;
					}
					else if (x >= 370 && x <= 572 && y >= 414 && y <= 619) { // on step6 button
						step[5] = true;
					}
					else if (x >= 687 && x <= 937 && y >= 445 && y <= 592) { // on step7 button
						step[6] = true;
					}
					else if (x >= 1019 && x <= 1271 && y >= 414 && y <= 618) { // on step8 button
						step[7] = true;
					}
					else {
						backbutton= false;
						nextbutton = false;
						mainmenubutton = false;
						step[0] = false;
						step[1] = false;
						step[2] = false;
						step[3] = false;
						step[4] = false;
						step[5] = false;
						step[6] = false;
						step[7] = false;
					}
					break;
				}
			}
			if (backbutton == true) { // on back button
				stretchTextureEx(renderer, 20, 55, 65, 65, arrow, 0);
			}
			else if (nextbutton == true) { // on next button
				stretchTextureEx(renderer, 1070, 55, 65, 65, arrow, 0);
			}
			else if (mainmenubutton == true) { // on mainmenu button
				stretchTextureEx(renderer, 910, 667, 65, 65, arrow, 0);
			}
			else if (step[0] == true) { // on step1 button
				stretchTextureEx(renderer, 123, 304, 65, 65, arrow, -90);
			}
			else if (step[1] == true) { // on step2 button
				stretchTextureEx(renderer, 431, 304, 65, 65, arrow, -90);
			}
			else if (step[2] == true) { // on step3 button
				stretchTextureEx(renderer, 759, 304, 65, 65, arrow, -90);
			}
			else if (step[3] == true) { // on step4 button
				stretchTextureEx(renderer, 1108, 304, 65, 65, arrow, -90);
			}
			else if (step[4] == true) { // on step5 button
				stretchTextureEx(renderer, 123, 610, 65, 65, arrow, -90);
			}
			else if (step[5] == true) { // on step6 button
				stretchTextureEx(renderer, 431, 610, 65, 65, arrow, -90);
			}
			else if (step[6] == true) { // on step7 button
				stretchTextureEx(renderer, 759, 610, 65, 65, arrow, -90);
			}
			else if (step[7] == true) { // on step8 button
				stretchTextureEx(renderer, 1108, 610, 65, 65, arrow, -90);
			}

		}
		else if (scene == in_stage) {

			Object * object=NULL;
			Object * tempt_object;
			cinf * contact;

			Bomb * bomb=NULL;
			bool explo = false;
			int bomb_population=0;

			float angle = 0;
			int population = 0;
			int material = 2; // wood
			int click_x, click_y;
			bool stage_prepare = true;
			bool stage_start = false;
			bool stage_start_button = false;
			bool back_to_stage_button = false;
			bool click_one_block = false;
			bool click_two_block = false;
			bool click_three_block = false;
			bool click_hexagon_block = false;
			bool click_polygon1_block = false;
			bool click_trapezoid_block = false;
			bool copy = false;
			char click_block = -1; // index of block

			next_click = false;

			int i, j, k;

			if(stage_number==1) map = loadTexture("resource\\stage1_back.png");
			else if(stage_number == 2) map = loadTexture("resource\\stage2_back.png");
			else if(stage_number == 3) map = loadTexture("resource\\stage3_back.png");
			else if(stage_number == 4) map = loadTexture("resource\\stage4_back.png");
			else if (stage_number == 5) map = loadTexture("resource\\stage5_back.png");
			else map = loadTexture("resource\\stage6_back.png");
			right_menu = loadTexture("resource\\stage_menu.png");
			back_to_stage = loadTexture("resource\\back_to_stage.png");
			description = loadTexture("resource\\running_description.png");
			start = loadTexture("resource\\start!.png");
			arrow= loadTexture("resource\\arrow.png");
			wood_one_block = loadTexture("resource\\wood_one.png");
			wood_two_block = loadTexture("resource\\wood_two.png");
			wood_three_block = loadTexture("resource\\wood_three.png");
			wood_hexagon_block = loadTexture("resource\\wood_hexagon.png");
			wood_polygon1_block = loadTexture("resource\\wood_polygon1.png");
			wood_trapezoid_block = loadTexture("resource\\wood_trapezoid.png");
			pre_wood_one_block = loadTexture("resource\\pre_wood_one.png");
			pre_wood_two_block = loadTexture("resource\\pre_wood_two.png");
			pre_wood_three_block = loadTexture("resource\\pre_wood_three.png");
			pre_wood_hexagon_block = loadTexture("resource\\pre_wood_hexagon.png");
			pre_wood_polygon1_block = loadTexture("resource\\pre_wood_polygon1.png");
			pre_wood_trapezoid_block = loadTexture("resource\\pre_wood_trapezoid.png");
			metal_one_block = loadTexture("resource\\metal_one.png");
			metal_two_block = loadTexture("resource\\metal_two.png");
			metal_three_block = loadTexture("resource\\metal_three.png");
			metal_hexagon_block = loadTexture("resource\\metal_hexagon.png");
			metal_polygon1_block = loadTexture("resource\\metal_polygon1.png");
			metal_trapezoid_block = loadTexture("resource\\metal_trapezoid.png");
			pre_metal_one_block=loadTexture("resource\\pre_metal_one.png");
			pre_metal_two_block=loadTexture("resource\\pre_metal_two.png");
			pre_metal_three_block=loadTexture("resource\\pre_metal_three.png");
			pre_metal_hexagon_block=loadTexture("resource\\pre_metal_hexagon.png");
			pre_metal_polygon1_block=loadTexture("resource\\pre_metal_polygon1.png");
			pre_metal_trapezoid_block=loadTexture("resource\\pre_metal_trapezoid.png");
			rock_one_block = loadTexture("resource\\rock_one.png");
			rock_two_block = loadTexture("resource\\rock_two.png");
			rock_three_block = loadTexture("resource\\rock_three.png");
			rock_hexagon_block = loadTexture("resource\\rock_hexagon.png");
			rock_polygon1_block = loadTexture("resource\\rock_polygon1.png");
			rock_trapezoid_block = loadTexture("resource\\rock_trapezoid.png");
			pre_rock_one_block = loadTexture("resource\\pre_rock_one.png");
			pre_rock_two_block = loadTexture("resource\\pre_rock_two.png");
			pre_rock_three_block = loadTexture("resource\\pre_rock_three.png");
			pre_rock_hexagon_block = loadTexture("resource\\pre_rock_hexagon.png");
			pre_rock_polygon1_block = loadTexture("resource\\pre_rock_polygon1.png");
			pre_rock_trapezoid_block = loadTexture("resource\\pre_rock_trapezoid.png");
			explosion1 = loadTexture("resource\\explosion1.png");
			explosion2 = loadTexture("resource\\explosion2.png");
			explosion3 = loadTexture("resource\\explosion3.png");
			explosion4 = loadTexture("resource\\explosion4.png");
			explosion5 = loadTexture("resource\\explosion5.png");
			explosion6 = loadTexture("resource\\explosion6.png");
			explosion7 = loadTexture("resource\\explosion7.png");
			explosion8 = loadTexture("resource\\explosion8.png");
			explosion9 = loadTexture("resource\\explosion9.png");
			explosion10 = loadTexture("resource\\explosion10.png");
			explosion11 = loadTexture("resource\\explosion11.png");
			explosion12 = loadTexture("resource\\explosion12.png");
			explosion13 = loadTexture("resource\\explosion13.png");
			explosion14 = loadTexture("resource\\explosion14.png");
			explosion15 = loadTexture("resource\\explosion15.png");
			explosion16 = loadTexture("resource\\explosion16.png");
			explosion17 = loadTexture("resource\\explosion17.png");
			bombimg = loadTexture("resource\\bombimg.png");
			cat = loadTexture("resource\\cat.png");
			stone = loadTexture("resource\\stone.png");
			large_soil= loadTexture("resource\\two_soil.png");
			ground_cone_small = loadTexture("resource\\soil_cone_small.png");
			hill = loadTexture("resource\\hill.png");
			line=loadTexture("resource\\line.png");
			question_round = loadTexture("resource\\round_question.png");
			five$ = loadTexture("resource\\5$.png");
			seven$ = loadTexture("resource\\7$.png");
			ten$ = loadTexture("resource\\10$.png");
			thirteen$ = loadTexture("resource\\13$.png");
			fifteen$ = loadTexture("resource\\15$.png");
			eighteen$ = loadTexture("resource\\18$.png");
			twenty$ = loadTexture("resource\\20$.png");
			twentythree$ = loadTexture("resource\\23$.png");
			twentysix$ = loadTexture("resource\\26$.png");
			twentyseven$ = loadTexture("resource\\27$.png");
			thirty$ = loadTexture("resource\\30$.png");
			thirtysix$ = loadTexture("resource\\36$.png");
			redline = loadTexture("resource\\redline.png");
			velocity_arrow = loadTexture("resource\\velocity_arrow.png");
			star = loadTexture("resource\\star.png");
			blood_dot = loadTexture("resource\\blood_dot.png");
			stop_retry = loadTexture("resource\\stop_retry.png");
			pause = loadTexture("resource\\pause.png");
			normal_speed = loadTexture("resource\\normal_speed.png");
			x2speed = loadTexture("resource\\x2speed.png");
			speed_select = loadTexture("resource\\speed_select.png");
			running_end_menu = loadTexture("resource\\running_end_menu.png");
			back_to_home = loadTexture("resource\\back_to_home.png");
			next_arrow = loadTexture("resource\\next_arrow.png");
			re_arrow = loadTexture("resource\\re_arrow.png");

			float height;
			float timeduration;
			int earthquake_type;
			float amplitude;
			float frequency;
			float starttime;
			float endtime;
			int budget;
			int initial_budget;
			bool description_on = false;

			int total_frame_start;
			int total_frame_end;
			float total_time;
			int total_delay_time;
			
			/*================map edition================*/
			map_editor(&object, &population,&budget, &height, &starttime, &timeduration, &endtime, &earthquake_type, &amplitude, &frequency, stage_number, step_number, &explo, &bomb_population, &bomb);
			/*================map edition================*/
			initial_budget = budget;

			if (retry == true) { // if retry button was clicked
				if (population != 0) {
					for (i = 0; i < population; i++) {
						for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
						free(object[i].shape->vertex); free(object[i].shape->normal);
						free(object[i].shape->vertex_contact_index);
						free(object[i].shape->vertex_contact);
						free(object[i].shape);
					}
					free(object);
				}
				population = pre_population;
				budget = pre_budget;
				object = (Object *)malloc(sizeof(Object)*population);
				for (i = 0; i < population; i++) set_object(&object[i], pre_object[i].shape->number, pre_object[i].material.number, pre_object[i].x[0]*trans, pre_object[i].x[1]*trans, pre_object[i].v[0] * trans, pre_object[i].v[1] * trans, pre_object[i].a[0] * trans, pre_object[i].a[1] * trans, pre_object[i].th, pre_object[i].w, pre_object[i].alp, pre_object[i].F[0], pre_object[i].F[1]);
				if (population != 0) {
					for (i = 0; i < population; i++) {
						for (j = 0; j < pre_object[i].shape->vertex_num; j++) { free(pre_object[i].shape->vertex[j]); free(pre_object[i].shape->normal[j]); }
						free(pre_object[i].shape->vertex); free(pre_object[i].shape->normal);
						free(pre_object[i].shape->vertex_contact_index);
						free(pre_object[i].shape->vertex_contact);
						free(pre_object[i].shape);
					}
					free(pre_object);
				}
				retry = false;
			}

			for (i = 0; i < population; i++) reassign_vertex(&object[i]);

			while (stage_prepare == true && quit == false) {
				total_frame_start = SDL_GetPerformanceCounter();

				drawTexture(renderer, 0, 0, map);
				drawTexture(renderer, 0, height, line);
				drawTexture(renderer, 1004, 0, right_menu);

				if (description_on == false) {
					sprintf(budget_string, "budget  %d$", budget);
					text_surface = TTF_RenderText_Blended(font40, budget_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1080, 530, text);
					drawTexture(renderer, 1080, 530, text);
					drawTexture(renderer, 1080, 530, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
				}

				drawTexture(renderer, 1304, 40, question_round);
				if (description_on == false)drawTexture(renderer, 1108, 709, back_to_stage);
				if (description_on == true)drawTexture(renderer, 1043, 91, description);
				if (description_on == false) {
					drawTexture(renderer, 1088, 622, start);
					if(budget<0) drawTexture(renderer, 1088, 622, redline);
				}
				if (description_on == false) {
					if (material == 2) {
						stretchTextureEx(renderer, 1124, 100, Length*trans, Length*trans, wood_one_block, 0);
						drawTexture(renderer, 1124, 145, five$);
					}
					else if (material == 3) {
						stretchTextureEx(renderer, 1124, 100, Length*trans, Length*trans, metal_one_block, 0);
						drawTexture(renderer, 1114, 145, ten$);
					}
					else if (material == 1) {
						stretchTextureEx(renderer, 1124, 100, Length*trans, Length*trans, rock_one_block, 0);
						drawTexture(renderer, 1124, 145, seven$);
					}
				}
				if (description_on == false) {
					if (material == 2) {
						stretchTextureEx(renderer, 1234, 100, 2 * Length*trans, Length*trans, wood_two_block, 0);
						drawTexture(renderer, 1244, 145, ten$);
					}
					else if (material == 3) {
						stretchTextureEx(renderer, 1234, 100, 2 * Length*trans, Length*trans, metal_two_block, 0);
						drawTexture(renderer, 1244, 145, twenty$);
					}
					else if (material == 1) {
						stretchTextureEx(renderer, 1234, 100, 2 * Length*trans, Length*trans, rock_two_block, 0);
						drawTexture(renderer, 1244, 145, fifteen$);
					}
				}
				if (description_on == false) {
					if (material == 2) {
						stretchTextureEx(renderer, 1084, 221, 3 * Length*trans, Length*trans, wood_three_block, 0);
						drawTexture(renderer, 1114, 266, fifteen$);
					}
					else if (material == 3) {
						stretchTextureEx(renderer, 1084, 221, 3 * Length*trans, Length*trans, metal_three_block, 0);
						drawTexture(renderer, 1114, 266, thirty$);
					}
					else if (material == 1) {
						stretchTextureEx(renderer, 1084, 221, 3 * Length*trans, Length*trans, rock_three_block, 0);
						drawTexture(renderer, 1114, 266, twentythree$);
					}
				}
				if (description_on == false) {
					if (material == 2) {
						stretchTextureEx_revise(renderer, 1235, 192, 2*Length*trans, Length*trans*sqrt(3) + 1, Length*trans, Length*trans*sqrt(3) / 2.0, wood_hexagon_block, 0);
						drawTexture(renderer, 1244, 266, thirteen$);
					}
					else if (material == 3) {
						stretchTextureEx_revise(renderer, 1235, 192, 2 * Length*trans, Length*trans*sqrt(3) + 1, Length*trans, Length*trans*sqrt(3) / 2.0, metal_hexagon_block, 0);
						drawTexture(renderer, 1244, 266, twentysix$);
					}
					else if (material == 1) {
						stretchTextureEx_revise(renderer, 1235, 192, 2 * Length*trans, Length*trans*sqrt(3) + 1, Length*trans, Length*trans*sqrt(3)/2.0, rock_hexagon_block, 0);
						drawTexture(renderer, 1244, 266, twenty$);
					}
				}
				if (description_on == false) {
					if (material == 2) {
						stretchTextureEx_revise(renderer, 1104, 308, 2 * Length*trans, 2 * Length*trans, 19 * Length*trans / 21.0, 23 * Length*trans / 21.0, wood_polygon1_block, 0);
						drawTexture(renderer, 1114, 387, eighteen$);
					}
					else if (material == 3) {
						stretchTextureEx_revise(renderer, 1104, 308, 2 * Length*trans, 2 * Length*trans, 19 * Length*trans / 21.0, 23 * Length*trans / 21.0, metal_polygon1_block, 0);
						drawTexture(renderer, 1114, 387, thirtysix$);
					}
					else if (material == 1) {
						stretchTextureEx_revise(renderer, 1104, 308, 2 * Length*trans, 2 * Length*trans, 19 * Length*trans / 21.0, 23 * Length*trans / 21.0, rock_polygon1_block, 0);
						drawTexture(renderer, 1114, 387, twentyseven$);
					}
				}
				if (description_on == false) {
					if (material == 2) {
						stretchTextureEx_revise(renderer, 1235, 347, 2 * Length*trans, sqrt(3)*Length*trans / 2.0, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), wood_trapezoid_block, 0);
						drawTexture(renderer, 1245, 387, ten$);
					}
					else if (material == 3) {
						stretchTextureEx_revise(renderer, 1235, 347, 2 * Length*trans, sqrt(3)*Length*trans / 2.0, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), metal_trapezoid_block, 0);
						drawTexture(renderer, 1245, 387, twenty$);
					}
					else if (material == 1) {
						stretchTextureEx_revise(renderer, 1235, 347, 2 * Length*trans, sqrt(3)*Length*trans / 2.0, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), rock_trapezoid_block, 0);
						drawTexture(renderer, 1245, 387, fifteen$);
					}
				}

				int x, y;
				x = event.motion.x;
				y = event.motion.y;

				int mouse_x, mouse_y;
/*				for (i = 0; i < population; i++) for (j = 0; j < object[i].shape->vertex_num; j++) {
					float vertex[2];
					vertex[0] = object[i].x[0] + object[i].shape->vertex[j][0];
					vertex[1] = object[i].x[1] + object[i].shape->vertex[j][1];
					stretchTextureEx(renderer, vertex[0] * trans - 2, vertex[1] * trans - 2, 5, 5, reddot, 0);
				}*/

				while (SDL_PollEvent(&event)) {
						if(event.type==SDL_QUIT) quit = true;

						if (event.type == SDL_MOUSEBUTTONDOWN) {
							if (event.button.button == SDL_BUTTON_LEFT) {
								int x, y;
								x = event.motion.x;
								y = event.motion.y;
								if (x >= 1108 && x <= 1355 && y >= 709 && y <= 736) { // click back to stage select
									stage_prepare = false;
									SDL_DestroyTexture(map);
									SDL_DestroyTexture(back_to_stage);
									SDL_DestroyTexture(description);
									SDL_DestroyTexture(start);
									SDL_DestroyTexture(right_menu);
									SDL_DestroyTexture(wood_one_block);
									SDL_DestroyTexture(wood_two_block);
									SDL_DestroyTexture(wood_three_block);
									SDL_DestroyTexture(wood_hexagon_block);
									SDL_DestroyTexture(wood_polygon1_block);
									SDL_DestroyTexture(wood_trapezoid_block);
									SDL_DestroyTexture(pre_wood_one_block);
									SDL_DestroyTexture(pre_wood_two_block);
									SDL_DestroyTexture(pre_wood_three_block);
									SDL_DestroyTexture(pre_wood_hexagon_block);
									SDL_DestroyTexture(pre_wood_polygon1_block);
									SDL_DestroyTexture(pre_wood_trapezoid_block);
									SDL_DestroyTexture(metal_one_block);
									SDL_DestroyTexture(metal_two_block);
									SDL_DestroyTexture(metal_three_block);
									SDL_DestroyTexture(metal_hexagon_block);
									SDL_DestroyTexture(metal_polygon1_block);
									SDL_DestroyTexture(metal_trapezoid_block);
									SDL_DestroyTexture(pre_metal_one_block);
									SDL_DestroyTexture(pre_metal_two_block);
									SDL_DestroyTexture(pre_metal_three_block);
									SDL_DestroyTexture(pre_metal_hexagon_block);
									SDL_DestroyTexture(pre_metal_polygon1_block);
									SDL_DestroyTexture(pre_metal_trapezoid_block);
									SDL_DestroyTexture(rock_one_block);
									SDL_DestroyTexture(rock_two_block);
									SDL_DestroyTexture(rock_three_block);
									SDL_DestroyTexture(rock_hexagon_block);
									SDL_DestroyTexture(rock_polygon1_block);
									SDL_DestroyTexture(rock_trapezoid_block);
									SDL_DestroyTexture(pre_rock_one_block);
									SDL_DestroyTexture(pre_rock_two_block);
									SDL_DestroyTexture(pre_rock_three_block);
									SDL_DestroyTexture(pre_rock_hexagon_block);
									SDL_DestroyTexture(pre_rock_polygon1_block);
									SDL_DestroyTexture(pre_rock_trapezoid_block);
									SDL_DestroyTexture(explosion1);
									SDL_DestroyTexture(explosion2);
									SDL_DestroyTexture(explosion3);
									SDL_DestroyTexture(explosion4);
									SDL_DestroyTexture(explosion5);
									SDL_DestroyTexture(explosion6);
									SDL_DestroyTexture(explosion7);
									SDL_DestroyTexture(explosion8);
									SDL_DestroyTexture(explosion9);
									SDL_DestroyTexture(explosion11);
									SDL_DestroyTexture(explosion10);
									SDL_DestroyTexture(explosion12);
									SDL_DestroyTexture(explosion13);
									SDL_DestroyTexture(explosion14);
									SDL_DestroyTexture(explosion15);
									SDL_DestroyTexture(explosion16);
									SDL_DestroyTexture(explosion17);
									SDL_DestroyTexture(bombimg);
									SDL_DestroyTexture(cat);
									SDL_DestroyTexture(stone);
									SDL_DestroyTexture(large_soil);
									SDL_DestroyTexture(ground_cone_small);
									SDL_DestroyTexture(hill);
									SDL_DestroyTexture(line);
									SDL_DestroyTexture(question_round);
									SDL_DestroyTexture(five$);
									SDL_DestroyTexture(seven$);
									SDL_DestroyTexture(ten$);
									SDL_DestroyTexture(thirteen$);
									SDL_DestroyTexture(fifteen$);
									SDL_DestroyTexture(eighteen$);
									SDL_DestroyTexture(twenty$);
									SDL_DestroyTexture(twentythree$);
									SDL_DestroyTexture(twentysix$);
									SDL_DestroyTexture(twentyseven$);
									SDL_DestroyTexture(thirty$);
									SDL_DestroyTexture(thirtysix$);
									SDL_DestroyTexture(redline);
									SDL_DestroyTexture(velocity_arrow);
									SDL_DestroyTexture(star);
									SDL_DestroyTexture(blood_dot);
									SDL_DestroyTexture(stop_retry);
									SDL_DestroyTexture(pause);
									SDL_DestroyTexture(normal_speed);
									SDL_DestroyTexture(x2speed);
									SDL_DestroyTexture(speed_select);
									SDL_DestroyTexture(running_end_menu);
									SDL_DestroyTexture(back_to_home);
									SDL_DestroyTexture(next_arrow);
									SDL_DestroyTexture(re_arrow);
									if(stage_number==1) scene = stage_select1;
									else if (stage_number == 2) scene = stage_select2;
									else if (stage_number == 3) scene = stage_select3;
									else if (stage_number == 4) scene = stage_select4;
									else if (stage_number == 5) scene = stage_select5;
									else if (stage_number == 6) scene = stage_select6;
									if (population != 0) {
										for (i = 0; i < population; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
											free(object[i].shape->vertex); free(object[i].shape->normal);
											free(object[i].shape->vertex_contact_index);
											free(object[i].shape->vertex_contact);
											free(object[i].shape);
										}
										free(object);
									}
									population = 0;
									if (bomb_population != 0) free(bomb);
									bomb_population = 0;
									Mix_PlayChannel(-1, click_sound, 0);
									Mix_HaltMusic();
									Mix_FadeInMusic(main_music, -1, 1000);
								}
								else if (x >= 1088 && x <= 1307 && y >= 622 && y <= 670 && budget>=0) { // click start button
									stage_start = true;
									stage_prepare = false;
									SDL_DestroyTexture(back_to_stage);
									SDL_DestroyTexture(description);
									SDL_DestroyTexture(start);
									SDL_DestroyTexture(question_round);
									SDL_DestroyTexture(five$);
									SDL_DestroyTexture(seven$);
									SDL_DestroyTexture(ten$);
									SDL_DestroyTexture(thirteen$);
									SDL_DestroyTexture(fifteen$);
									SDL_DestroyTexture(eighteen$);
									SDL_DestroyTexture(twenty$);
									SDL_DestroyTexture(twentythree$);
									SDL_DestroyTexture(twentysix$);
									SDL_DestroyTexture(twentyseven$);
									SDL_DestroyTexture(thirty$);
									SDL_DestroyTexture(thirtysix$);
									SDL_DestroyTexture(redline);
									SDL_DestroyTexture(velocity_arrow);
									contact = (cinf *)malloc(sizeof(cinf) * 4 * population);
									for (i = 0; i < 4 * population; i++) {
										contact[i].run = false;
										contact[i].normal_impulse = 0;
										contact[i].tangent_impulse = 0;
									}
									pre_object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population; i++) set_object(&pre_object[i], object[i].shape->number, object[i].material.number, object[i].x[0]*trans, object[i].x[1]*trans, object[i].v[0] * trans, object[i].v[1] * trans, object[i].a[0], object[i].a[1], object[i].th, object[i].w, object[i].alp, object[i].F[0], object[i].F[1]);
									pre_population = population;
									pre_budget = budget;
								}
								else if (x >= 1124 && x <= 1124 + Length*trans && y >= 100 && y <= 100 + Length*trans) { // click one block
									click_one_block = true;
								}
								else if (x >= 1234 && x <= 1234 + 2 * Length*trans && y >= 100 && y <= 100 + Length*trans) { // click two block
									click_two_block = true;
								}
								else if (x >= 1084 && x <= 1084 + 3 * Length*trans && y >= 216 && y <= 216 + Length*trans) { // click three block
									click_three_block = true;
								}
								else if (x >= 1235 && x <= 1235 + 2*Length*trans && y >= 192 && y <= 192 + Length*trans*sqrt(3)) { // click hexagon block
									click_hexagon_block = true;
								}
								else if (x >= 1104 && x <= 1104 + 2 * Length*trans && y >= 308 && y <= 308 + 2 * Length*trans) { // click polygon1 block
									click_polygon1_block = true;
								}
								else if (x>=1235 && x<=1235+2*Length*trans && y>=347 && y<=347+Length*trans*sqrt(3)/2.0) {
									click_trapezoid_block = true;
								}
								else {
									for (i = 0; i < population; i++) if (click_perception(mouse_x / trans, mouse_y / trans, &object[i]) < 0 && (object[i].material.number!=99&& object[i].material.number != 98 && object[i].material.number != 4 && object[i].material.number !=5 && object[i].material.number != 6 && object[i].material.number != 97)) {
										if (copy == true) {
											click_block = i; click_x = mouse_x; click_y = mouse_y;

											if (object[click_block].shape->number == 1) {
												if (object[click_block].material.number == 2) budget = budget - 5;
												else if (object[click_block].material.number == 3) budget = budget - 10;
												else if (object[click_block].material.number == 1) budget = budget - 7;
											}
											else if (object[click_block].shape->number == 2) {
												if (object[click_block].material.number == 2) budget = budget - 10;
												else if (object[click_block].material.number == 3) budget = budget - 20;
												else if (object[click_block].material.number == 1) budget = budget - 15;
											}
											else if (object[click_block].shape->number == 3) {
												if (object[click_block].material.number == 2) budget = budget - 15;
												else if (object[click_block].material.number == 3) budget = budget - 30;
												else if (object[click_block].material.number == 1) budget = budget - 23;
											}
											else if (object[click_block].shape->number == 4) {
												if (object[click_block].material.number == 2) budget = budget - 3;
												else if (object[click_block].material.number == 3) budget = budget - 6;
												else if (object[click_block].material.number == 1) budget = budget - 5;
											}
											else if (object[click_block].shape->number == 5) {
												if (object[click_block].material.number == 2) budget = budget - 3;
												else if (object[click_block].material.number == 3) budget = budget - 6;
												else if (object[click_block].material.number == 1) budget = budget - 5;
											}
											else if (object[click_block].shape->number == 6) {
												if (object[click_block].material.number == 2) budget = budget - 10;
												else if (object[click_block].material.number == 3) budget = budget - 20;
												else if (object[click_block].material.number == 1) budget = budget - 15;
											}
											else if (object[click_block].shape->number == 8) {
												if (object[click_block].material.number == 2) budget = budget - 13;
												else if (object[click_block].material.number == 3) budget = budget - 26;
												else if (object[click_block].material.number == 1) budget = budget - 20;
											}
											else if (object[click_block].shape->number == 9) {
												if (object[click_block].material.number == 2) budget = budget - 18;
												else if (object[click_block].material.number == 3) budget = budget - 36;
												else if (object[click_block].material.number == 1) budget = budget - 27;
											}

											tempt_object = (Object *)malloc(sizeof(Object)*population);
											for (i = 0; i < population; i++) set_object(&tempt_object[i], object[i].shape->number, object[i].material.number, object[i].x[0]*trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
											if (population != 0) {
												for (i = 0; i < population; i++) {
													for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
													free(object[i].shape->vertex); free(object[i].shape->normal);
													free(object[i].shape->vertex_contact_index);
													free(object[i].shape->vertex_contact);
													free(object[i].shape);
												}
												free(object);
											}
											population = population + 1;
											object = (Object *)malloc(sizeof(Object)*population);
											for (i = 0; i < population - 1; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
											set_object(&object[population - 1], object[click_block].shape->number, object[click_block].material.number, object[click_block].x[0] * trans, object[click_block].x[1] * trans, 0, 0, 0, 0, object[click_block].th, 0, 0, 0, 0);
											if (population - 1 != 0) {
												for (i = 0; i < population - 1; i++) {
													for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
													free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
													free(tempt_object[i].shape->vertex_contact_index);
													free(tempt_object[i].shape->vertex_contact);
													free(tempt_object[i].shape);
												}
												free(tempt_object);
											}
											for (i = 0; i < population; i++) reassign_vertex(&object[i]);
										}
										else click_block = i; click_x = mouse_x; click_y = mouse_y;
									}
								}
							}
							else if (event.button.button == SDL_BUTTON_RIGHT) {
								for (k = 0; k < population; k++) if (click_perception(mouse_x/trans, mouse_y / trans, &object[k]) < 0 && (object[k].material.number != 99&& object[k].material.number != 98 && object[k].material.number != 4 && object[k].material.number != 5 && object[k].material.number != 6 && object[k].material.number != 97) && click_block == -1) {
									int i, j;

									if (object[k].shape->number == 1) {
										if (object[k].material.number == 2) budget = budget + 5;
										else if (object[k].material.number == 3) budget = budget + 10;
										else if (object[k].material.number == 1) budget = budget + 7;
									}
									else if (object[k].shape->number == 2) {
										if (object[k].material.number == 2) budget = budget + 10;
										else if (object[k].material.number == 3) budget = budget + 20;
										else if (object[k].material.number == 1) budget = budget + 15;
									}
									else if (object[k].shape->number == 3) {
										if (object[k].material.number == 2) budget = budget + 15;
										else if (object[k].material.number == 3) budget = budget + 30;
										else if (object[k].material.number == 1) budget = budget + 23;
									}
									else if (object[k].shape->number == 4) {
										if (object[k].material.number == 2) budget = budget + 3;
										else if (object[k].material.number == 3) budget = budget + 6;
										else if (object[k].material.number == 1) budget = budget + 5;
									}
									else if (object[k].shape->number == 5) {
										if (object[k].material.number == 2) budget = budget + 3;
										else if (object[k].material.number == 3) budget = budget + 6;
										else if (object[k].material.number == 1) budget = budget + 5;
									}
									else if (object[k].shape->number == 6) {
										if (object[k].material.number == 2) budget = budget + 10;
										else if (object[k].material.number == 3) budget = budget + 20;
										else if (object[k].material.number == 1) budget = budget + 15;
									}
									else if (object[k].shape->number == 8) {
										if (object[k].material.number == 2) budget = budget + 13;
										else if (object[k].material.number == 3) budget = budget + 26;
										else if (object[k].material.number == 1) budget = budget + 20;
									}
									else if (object[k].shape->number == 9) {
										if (object[k].material.number == 2) budget = budget + 18;
										else if (object[k].material.number == 3) budget = budget + 36;
										else if (object[k].material.number == 1) budget = budget + 27;
									}
									population = population - 1;
									tempt_object = (Object *)malloc(sizeof(Object)*population);
									i = 0; j = 0;
									for (i = 0; i < population + 1; i++) if (i != k) {
										set_object(&tempt_object[j], object[i].shape->number, object[i].material.number, object[i].x[0]*trans, object[i].x[1]*trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
										j = j + 1;
									}
									for (i = 0; i < population + 1; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
										free(object[i].shape->vertex); free(object[i].shape->normal);
										free(object[i].shape->vertex_contact_index);
										free(object[i].shape->vertex_contact);
										free(object[i].shape);
									}
									free(object);
									object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0]*trans, tempt_object[i].x[1]*trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
									for (i = 0; i < population; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
										free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
										free(tempt_object[i].shape->vertex_contact_index);
										free(tempt_object[i].shape->vertex_contact);
										free(tempt_object[i].shape);
									}
									free(tempt_object);
									for (i = 0; i < population; i++) reassign_vertex(&object[i]);
									Mix_PlayChannel(-1, erase, 0);
									break;
								}
							}
						}

						if (event.type == SDL_MOUSEMOTION) {
							int x, y;
							x = event.motion.x;
							y = event.motion.y;
							mouse_x = x;
							mouse_y = y;

							if (x >= 1108 && x <= 1355 && y >= 709 && y <= 736) { // on back to stage button
								back_to_stage_button = true;
							}
							else if (x >= 1304 && x <= 1341 && y >= 40 && y <= 77) { // on question round
								description_on = true;
							}
							else if (x >= 1088 && x <= 1307 && y >= 622 && y <= 670 && budget >= 0) {
								stage_start_button = true;
							}
							else {
								back_to_stage_button = false;
								description_on = false;
								stage_start_button = false;
							}
							if(click_block!=-1) {
								object[click_block].x[0] = object[click_block].x[0] + (mouse_x - click_x)/trans;
								object[click_block].x[1] = object[click_block].x[1] + (mouse_y - click_y)/trans;
								click_x = mouse_x; click_y = mouse_y;
							}
						}

						if (event.type == SDL_MOUSEBUTTONUP) {
							if (click_one_block == true|| click_two_block == true|| click_three_block == true || click_hexagon_block == true || click_polygon1_block==true || click_trapezoid_block==true) {
								if (mouse_x < 1004) {
									int i, j;
									tempt_object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population; i++) set_object(&tempt_object[i], object[i].shape->number, object[i].material.number, object[i].x[0]*trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
									if (population != 0) {
										for (i = 0; i < population; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
											free(object[i].shape->vertex); free(object[i].shape->normal);
											free(object[i].shape->vertex_contact_index);
											free(object[i].shape->vertex_contact);
											free(object[i].shape);
										}
										free(object);
									}
									population = population + 1;
									object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population - 1; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
									if (click_one_block == true) {
										click_one_block = false;
										set_object(&object[population-1], 1, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi/180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 5;
										else if(material == 3) budget = budget - 10;
										else if (material == 1) budget = budget - 7;
									}
									else if (click_two_block == true) {
										click_two_block = false;
										set_object(&object[population - 1], 2, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 10;
										else if (material == 3) budget = budget - 20;
										else if (material == 1) budget = budget - 15;
									}
									else if (click_three_block == true) {
										click_three_block = false;
										set_object(&object[population - 1], 3, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 15;
										else if (material == 3) budget = budget - 30;
										else if (material == 1) budget = budget - 23;
									}
									else if (click_hexagon_block == true) {
										click_hexagon_block = false;
										set_object(&object[population - 1], 8, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 13;
										else if (material == 3) budget = budget - 26;
										else if (material == 1) budget = budget - 20;
									}
									else if (click_polygon1_block == true) {
										click_polygon1_block = false;
										set_object(&object[population - 1], 9, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 18;
										else if (material == 3) budget = budget - 36;
										else if (material == 1) budget = budget - 27;
									}
									else if (click_trapezoid_block == true) {
										click_trapezoid_block = false;
										set_object(&object[population - 1], 6, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 10;
										else if (material == 3) budget = budget - 20;
										else if (material == 1) budget = budget - 15;
									}
									if (population-1 != 0) {
										for (i = 0; i < population-1; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
											free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
											free(tempt_object[i].shape->vertex_contact_index);
											free(tempt_object[i].shape->vertex_contact);
											free(tempt_object[i].shape);
										}
										free(tempt_object);
									}
									for (i = 0; i < population;i++) reassign_vertex(&object[i]);
									angle = 0;
									if(material == 2) Mix_PlayChannel(-1, hammering_wood, 0);
									else if(material == 3) Mix_PlayChannel(-1, hammering_iron, 0);
									else if (material == 1)Mix_PlayChannel(-1, hammering_brick, 0);
								}
								else if (mouse_x >= 1004) {
									click_one_block = false;
									click_two_block = false;
									click_three_block = false;
									click_hexagon_block = false;
									click_polygon1_block = false;
									click_trapezoid_block = false;
									angle = 0;
									Mix_PlayChannel(-1, erase, 0);
								}
							}
							if (click_block != -1) {
								if (object[click_block].x[0] >= 1004/trans) {
									int i, j;

									if (object[click_block].shape->number == 1) {
										if (object[click_block].material.number == 2) budget = budget + 5;
										else if (object[click_block].material.number == 3) budget = budget + 10;
										else if (object[click_block].material.number == 1) budget = budget + 7;
									}
									else if (object[click_block].shape->number == 2) {
										if (object[click_block].material.number == 2) budget = budget + 10;
										else if (object[click_block].material.number == 3) budget = budget + 20;
										else if (object[click_block].material.number == 1) budget = budget + 15;
									}
									else if (object[click_block].shape->number == 3) {
										if (object[click_block].material.number == 2) budget = budget + 15;
										else if (object[click_block].material.number == 3) budget = budget + 30;
										else if (object[click_block].material.number == 1) budget = budget + 23;
									}
									else if (object[click_block].shape->number == 4) {
										if (object[click_block].material.number == 2) budget = budget + 3;
										else if (object[click_block].material.number == 3) budget = budget + 6;
										else if (object[click_block].material.number == 1) budget = budget + 5;
									}
									else if (object[click_block].shape->number == 5) {
										if (object[click_block].material.number == 2) budget = budget + 3;
										else if (object[click_block].material.number == 3) budget = budget + 6;
										else if (object[click_block].material.number == 1) budget = budget + 5;
									}
									else if (object[click_block].shape->number == 6) {
										if (object[click_block].material.number == 2) budget = budget + 10;
										else if (object[click_block].material.number == 3) budget = budget + 20;
										else if (object[click_block].material.number == 1) budget = budget + 15;
									}
									else if (object[click_block].shape->number == 8) {
										if (object[click_block].material.number == 2) budget = budget + 13;
										else if (object[click_block].material.number == 3) budget = budget + 26;
										else if (object[click_block].material.number == 1) budget = budget + 20;
									}
									else if (object[click_block].shape->number == 9) {
										if (object[click_block].material.number == 2) budget = budget + 18;
										else if (object[click_block].material.number == 3) budget = budget + 36;
										else if (object[click_block].material.number == 1) budget = budget + 27;
									}

									population = population - 1;
									tempt_object = (Object *)malloc(sizeof(Object)*population);
									i = 0; j = 0;
									for (i = 0; i < population + 1; i++) if (i != click_block) {
										set_object(&tempt_object[j], object[i].shape->number, object[i].material.number, object[i].x[0]*trans, object[i].x[1]*trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
										j = j + 1;
									}
									for (i = 0; i < population + 1; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
										free(object[i].shape->vertex); free(object[i].shape->normal);
										free(object[i].shape->vertex_contact_index);
										free(object[i].shape->vertex_contact);
										free(object[i].shape);
									}
									free(object);
									object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0]*trans, tempt_object[i].x[1]*trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
									for (i = 0; i < population; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
										free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
										free(tempt_object[i].shape->vertex_contact_index);
										free(tempt_object[i].shape->vertex_contact);
										free(tempt_object[i].shape);
									}
									free(tempt_object);
									Mix_PlayChannel(-1, erase, 0);
								}
								else {
									if (object[click_block].material.number == 2) Mix_PlayChannel(-1, hammering_wood, 0);
									else if (object[click_block].material.number == 3) Mix_PlayChannel(-1, hammering_iron, 0);
									else if (object[click_block].material.number == 1)Mix_PlayChannel(-1, hammering_brick, 0);
								}
								click_block = -1;
								for (i = 0; i < population; i++) reassign_vertex(&object[i]);
								angle = 0;
							}
						}

						if (event.type == SDL_KEYDOWN) {
							if (event.key.keysym.sym == SDLK_r) {
								if (click_one_block == true || click_two_block == true || click_three_block == true || click_hexagon_block == true || click_polygon1_block == true || click_trapezoid_block == true) { angle = angle - 15; Mix_PlayChannel(-1, gear, 0); }
								if (click_block != -1) { object[click_block].th = object[click_block].th - 15 * pi / 180; Mix_PlayChannel(-1, gear, 0); }
							}
							else if (event.key.keysym.sym == SDLK_t) {
								if (click_one_block == true || click_two_block == true || click_three_block == true || click_hexagon_block == true || click_polygon1_block == true || click_trapezoid_block == true) { angle = angle + 15; Mix_PlayChannel(-1, gear, 0); }
								if (click_block != -1) { object[click_block].th = object[click_block].th + 15 * pi / 180; Mix_PlayChannel(-1, gear, 0); }
							}
							else if (event.key.keysym.sym == SDLK_q) {
								material = material + 1;
								if (material == 4)material = 1;
							}
							else if (event.key.keysym.sym == SDLK_LCTRL) {
								copy = true;
							}
						}

						if (event.type == SDL_KEYUP) {
							if (event.key.keysym.sym == SDLK_LCTRL) {
								copy = false;
							}
						}
				}
				if (back_to_stage_button == true) stretchTextureEx(renderer, 1081, 708, 27, 27, arrow, 0);
				else if (stage_start_button == true) drawTexture(renderer, 1023, 613, arrow);

				if (click_one_block == true) {
					if(material == 2) stretchTextureEx(renderer, mouse_x - Length*trans / 2, mouse_y - Length*trans / 2, Length*trans, Length*trans, wood_one_block, angle);
					else if (material == 3) stretchTextureEx(renderer, mouse_x - Length*trans / 2, mouse_y - Length*trans / 2, Length*trans, Length*trans, metal_one_block, angle);
					else if (material == 1) stretchTextureEx(renderer, mouse_x - Length*trans / 2, mouse_y - Length*trans / 2, Length*trans, Length*trans, rock_one_block, angle);
				}
				else if (click_two_block == true) {
					if(material ==2) stretchTextureEx(renderer, mouse_x - Length*trans, mouse_y - Length*trans / 2, 2 * Length*trans, Length*trans, wood_two_block, angle);
					else if (material == 3) stretchTextureEx(renderer, mouse_x - Length*trans, mouse_y - Length*trans / 2, 2 * Length*trans, Length*trans, metal_two_block, angle);
					else if (material == 1) stretchTextureEx(renderer, mouse_x - Length*trans, mouse_y - Length*trans / 2, 2 * Length*trans, Length*trans, rock_two_block, angle);
				}
				else if (click_three_block == true) {
					if (material == 2) stretchTextureEx(renderer, mouse_x - 3 * Length*trans / 2, mouse_y - Length*trans / 2, 3 * Length*trans, Length*trans, wood_three_block, angle);
					else if(material == 3) stretchTextureEx(renderer, mouse_x - 3 * Length*trans / 2, mouse_y - Length*trans / 2, 3 * Length*trans, Length*trans, metal_three_block, angle);
					else if(material == 1) stretchTextureEx(renderer, mouse_x - 3 * Length*trans / 2, mouse_y - Length*trans / 2, 3 * Length*trans, Length*trans, rock_three_block, angle);
				}
				else if (click_hexagon_block == true) {
					if(material == 2) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans/2.0, 2*Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, wood_hexagon_block, angle);
					else if(material == 3) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, metal_hexagon_block, angle);
					else if (material == 1) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, rock_hexagon_block, angle);
				}
				else if (click_polygon1_block == true) {
					if(material == 2) stretchTextureEx_revise(renderer, mouse_x - 19.0*Length*trans/21.0, mouse_y - 23.0*Length*trans / 21.0, 2*Length*trans, 2*Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, wood_polygon1_block, angle);
					else if (material == 3) stretchTextureEx_revise(renderer, mouse_x - 19.0*Length*trans / 21.0, mouse_y - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, metal_polygon1_block, angle);
					else if (material == 1) stretchTextureEx_revise(renderer, mouse_x - 19.0*Length*trans / 21.0, mouse_y - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, rock_polygon1_block, angle);
				}
				else if (click_trapezoid_block == true) {
					if (material == 2) stretchTextureEx_revise(renderer, mouse_x-Length*trans, mouse_y-sqrt(3)*Length*trans/4.0-2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), wood_trapezoid_block, angle);
					else if (material == 3) stretchTextureEx_revise(renderer, mouse_x-Length*trans, mouse_y-sqrt(3)*Length*trans/4.0-2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), metal_trapezoid_block, angle);
					else if (material == 1) stretchTextureEx_revise(renderer, mouse_x-Length*trans, mouse_y-sqrt(3)*Length*trans/4.0-2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), rock_trapezoid_block, angle);
				}

				for (i = 0; i < population; i++) {
					if (object[i].shape->number == 1) {
						if(object[i].material.number == 2)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans -Length*trans / 2, Length*trans, Length*trans, wood_one_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 3)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, metal_one_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 1)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, rock_one_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, pre_wood_one_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, pre_metal_one_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, pre_rock_one_block, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 2) {
						if(object[i].material.number == 2) stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, wood_two_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 3)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, metal_two_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 1)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, rock_two_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, pre_wood_two_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, pre_metal_two_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, pre_rock_two_block, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 3) {
						if(object[i].material.number == 2) stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, wood_three_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 3)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, metal_three_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 1)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, rock_three_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, pre_wood_three_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, pre_metal_three_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, pre_rock_three_block, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 6) {
						if (object[i].material.number == 2) stretchTextureEx_revise(renderer, object[i].x[0] * trans -Length*trans, object[i].x[1] * trans -sqrt(3)*Length*trans/4.0-2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), wood_trapezoid_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 3) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0-2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), metal_trapezoid_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 1) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0-2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), rock_trapezoid_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_wood_trapezoid_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_metal_trapezoid_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_rock_trapezoid_block, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 8) {
						if (object[i].material.number == 2) stretchTextureEx_revise(renderer, object[i].x[0]*trans - Length*trans, object[i].x[1]*trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, wood_hexagon_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 3) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, metal_hexagon_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 1) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, rock_hexagon_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_wood_hexagon_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_metal_hexagon_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_rock_hexagon_block, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 9) {
						if (object[i].material.number == 2) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, wood_polygon1_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 3) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, metal_polygon1_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 1) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, rock_polygon1_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_wood_polygon1_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_metal_polygon1_block, object[i].th * 180 / pi);
						else if (object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_rock_polygon1_block, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 80) {
						stretchTextureEx(renderer, object[i].x[0] * trans - 49.96*Length*trans/40.0, object[i].x[1] * trans - 33.02*Length*trans/40.0, 101*Length*trans/40.0, 69.0*Length*trans / 40.0, stone, object[i].th * 180 / pi);
						if (!(object[i].v[0] == 0 && object[i].v[1] == 0)) {
							float x, y, theta, mag;
							x = object[i].v[0]; y = object[i].v[1];
							mag = sqrt(x*x + y*y);
							x = x / mag; y = y / mag;
							theta = asin(absolute(y));
							if (x >= 0 && y >= 0) theta = theta;
							else if (x >= 0 && y < 0) theta = 2 * pi - theta;
							else if (x < 0 && y >= 0) theta = pi - theta;
							else if (x < 0 && y < 0) theta = pi + theta;
							stretchTextureEx(renderer, object[i].x[0] * trans + x * 100-25, object[i].x[1] * trans + y * 100, 50, 21, velocity_arrow, theta * 180 / pi);
						}
					}
					else if (object[i].shape->number == 81) { //0.733511 0.759908
						stretchTextureEx_revise(renderer, object[i].x[0] * trans - 25.5*Length*trans / 40.0-3, object[i].x[1] * trans - 31.0*Length*trans / 40.0, 51 * Length*trans / 40.0, 62.0*Length*trans / 40.0,0.733511*Length*trans,0.759908*Length*trans, cat, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 97) {
						stretchTextureEx_revise(renderer, object[i].x[0] * trans - 2.946671f*Length*trans, object[i].x[1] * trans - 3.543418f*Length*trans, 250.0*Length*trans / 40.0, 279.0*Length*trans / 40.0, 117.809f*Length*trans / 40.0, 142.2465f*Length*trans / 40.0f, hill, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 98) {
						stretchTextureEx_revise(renderer, object[i].x[0] * trans - 77.5*Length*trans / 40.0, object[i].x[1] * trans - 4459.0*Length*trans / (219.0*40.0), 155.0*Length*trans / 40.0, 58.0*Length*trans / 40.0, 77.5*Length*trans / 40.0, 20.36*Length*trans / 40.0, ground_cone_small, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 99) {
						stretchTextureEx(renderer, object[i].x[0] * trans - 7 * Length*trans, object[i].x[1] * trans - 2*Length*trans, 14 * Length*trans, 4 * Length*trans, large_soil, object[i].th * 180 / pi);
					}
				}

				for (i = 0; i < bomb_population; i++) {
							stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0);
				}
/*				for (k = 1; k <= bomb_lenght_divide_num; k++) {
					for (j = 0; j < bomb_angle_divide; j++) {
						stretchTextureEx(renderer, (bomb[0].x[0] + k*bomb_length_divide*cos((j*2.0*pi) / bomb_angle_divide)) * trans - 2, (bomb[0].x[1] + k*bomb_length_divide*sin((j*2.0*pi) / bomb_angle_divide)) * trans - 2, 5, 5, reddot, 0);
					}
				}*/

				SDL_RenderPresent(renderer);

				total_frame_end = SDL_GetPerformanceCounter();
				total_time = (float)(total_frame_end - total_frame_start) / ((float)SDL_GetPerformanceFrequency());
				total_delay_time = (int)(((1 / fpss)*rendering_factor - total_time) * 1000);

				if (total_delay_time>0) SDL_Delay(total_delay_time);
				else SDL_Delay(1);

				if(stage_prepare != false && stage_start!=false) SDL_RenderClear(renderer);
			}
			if (stage_start == true && quit == false) {

				if (explo == false) {
					explo = true;
					Mix_PlayChannel(-1, explosion, 0);
					bomb_function(object, population, bomb, bomb_population);
				}

				time_ = 0;
				float accumulator = 0;
				int framestart = SDL_GetPerformanceCounter();
				int currenttime;
				float initial_accumulate;
				float max_height = FLT_MAX;
				bool end = false;
				bool clear = false;
				bool fade_out_earthquake_sound = false;
				char text_string[50];
				int star_framestart; int star_currenttime; float star_deltime = 0; float star_accumulate = 0;
				float rendering_factor;
				if (graphic_quality == 1) rendering_factor = 1.0;
				else if (graphic_quality == 0) rendering_factor = 2.0;
				else if (graphic_quality == -1) rendering_factor = 4.0;

				float star_vx[10] = { 2.0f,1.0f,-1.0f,5.0f,3.0f,-0.5f,-4.0f,0.9f,2.4f,-3.1f };
				float star_vy[10] = { 7.0f,3.0f,2.0f,5.0f,4.0f,4.6f,2.8f,1.2f,3.5f,4.8f };
				float star_x[10] = { 454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans };
				float star_y[10] = { 252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans };
				float star_th[10] = { 0.0f,1.0f,2.0f,-1.0f,-2.0f,1.2f,1.3f,-2.4f,1.0f,3.7f };
				float star_w[10] = { 0.1f,0.2f,0.3f,0.4f,-0.1f,-0.5f,-0.8f,3.5f,1.2f,-5.3f };

				bool cat_hurt = false;
				float cat_hurt_coordinate[2];
				int cat_framestart; int cat_currenttime; float cat_deltime = 0; float cat_accumulate = 0;
				float cat_vx[10] = { 2.0f,1.0f,-1.0f,5.0f,3.0f,-0.5f,-4.0f,0.9f,2.4f,-3.1f };
				float cat_vy[10] = { 7.0f,3.0f,2.0f,5.0f,4.0f,4.6f,2.8f,1.2f,3.5f,4.8f };
				float cat_x[10];
				float cat_y[10];
				float cat_th[10] = { 0.0f,1.0f,2.0f,-1.0f,-2.0f,1.2f,1.3f,-2.4f,1.0f,3.7f };
				float cat_w[10] = { 0.1f,0.2f,0.3f,0.4f,-0.1f,-0.5f,-0.8f,3.5f,1.2f,-5.3f };

				int total_frame_start;
				int total_frame_end;
				float total_time;
				int total_delay_time;

				if (earthquake_type != 0) {
					Mix_PlayChannel(-1, earthquake_sound, 0);
				}

				if(speed == 0) Mix_Pause(-1);

				while (quit == false && stage_start == true) {
					total_frame_start = SDL_GetPerformanceCounter();

					currenttime= SDL_GetPerformanceCounter();
					accumulator = accumulator + (float)(currenttime - framestart)/((float)SDL_GetPerformanceFrequency());
					framestart= SDL_GetPerformanceCounter();

					initial_accumulate = accumulator;
					if (accumulator > 0.2) accumulator = 0.2;

					float max_speed_BB = -FLT_MAX;
					float max_speed_BE = -FLT_MAX;

					while (accumulator>(1/fpss)*rendering_factor && time_<endtime) {
						if (speed != 0) {
							if (time_ < starttime) {
								one_term(object, population, contact);
								time_ = time_ + (1 / fpss);
							}
							else if (time_ < timeduration) one_term_with_earthquake(object, population, contact, earthquake_type, &time_, amplitude, frequency);
							else if (time_ < endtime) {
								one_term(object, population, contact);
								time_ = time_ + (1 / fpss);
							}
							accumulator = accumulator - (1 / fpss)/speed;
							cat_hurt_check(object, population, contact, &cat_hurt, cat_hurt_coordinate);
							if (cat_hurt == true) break;

							/* collide sound detect */
							for (i = 0; i < 4 * population; i++) {
								int numA;
								int numB;
								if (contact[i].run == true) {
									numA = contact[i].numA;
									numB = contact[i].numB;
									if (object[numA].material.number == 99 || object[numB].material.number == 99) {
										if (contact[i].velocity_bias * 50 > max_speed_BE) max_speed_BE = contact[i].velocity_bias * 50;
									}
									else {
										if (contact[i].velocity_bias * 50 > max_speed_BB) max_speed_BB = contact[i].velocity_bias * 50;
									}
								}
							}
							/* collide sound detect */
						}
						else {
							accumulator = accumulator - (1 / fpss);
						}
					}

					if (time_ < endtime) {
						if (max_speed_BB > 0) {
							float ratio = (max_speed_BB - sound_threshold_speed) / sound_max_speed;
							if (ratio > 0) {
								if (ratio > 1) ratio = 1;
								Mix_VolumeChunk(collide_BB, (int)(chunk_sound * ratio));
								Mix_PlayChannel(-1, collide_BB, 0);
							}
						}
						if (max_speed_BE > 0) {
							float ratio = (max_speed_BE - sound_threshold_speed) / sound_max_speed;
							if (ratio > 0) {
								if (ratio > 1) ratio = 1;
								Mix_VolumeChunk(collide_BE, (int)(chunk_sound * ratio));
								Mix_PlayChannel(-1, collide_BE, 0);
							}
						}
					}

					if (time_ > timeduration && fade_out_earthquake_sound == false) {
						Mix_FadeOutChannel(-1, 1000);
						fade_out_earthquake_sound = true;
					}

					if (time_ < endtime) {
						while (SDL_PollEvent(&event)) {
							switch (event.type) {
							case SDL_QUIT: quit = true; break;
							case SDL_MOUSEBUTTONDOWN: {
								if (event.button.button == SDL_BUTTON_LEFT) {
									int x, y;
									x = event.motion.x;
									y = event.motion.y;
									if (x > 1117 && x < 1160 && y>688 && y < 738) { speed = 0; Mix_Pause(-1); Mix_PlayChannel(-1, click_sound, 0); }
									else if (x > 1186 && x < 1234 && y>688 && y < 739) { speed = 1; Mix_Resume(-1); Mix_PlayChannel(-1, click_sound, 0); }
									else if (x > 1255 && x < 1347 && y>688 && y < 739) { speed = 2; Mix_Resume(-1); Mix_PlayChannel(-1, click_sound, 0); }
									else if (x >= 1046 && x < 1095 && y >= 685 && y < 738) { // click retry button
										stage_start = false;
										stage_prepare = true;
										SDL_DestroyTexture(map);
										SDL_DestroyTexture(right_menu);
										SDL_DestroyTexture(arrow);
										SDL_DestroyTexture(wood_one_block);
										SDL_DestroyTexture(wood_two_block);
										SDL_DestroyTexture(wood_three_block);
										SDL_DestroyTexture(wood_hexagon_block);
										SDL_DestroyTexture(wood_polygon1_block);
										SDL_DestroyTexture(wood_trapezoid_block);
										SDL_DestroyTexture(pre_wood_one_block);
										SDL_DestroyTexture(pre_wood_two_block);
										SDL_DestroyTexture(pre_wood_three_block);
										SDL_DestroyTexture(pre_wood_hexagon_block);
										SDL_DestroyTexture(pre_wood_polygon1_block);
										SDL_DestroyTexture(pre_wood_trapezoid_block);
										SDL_DestroyTexture(metal_one_block);
										SDL_DestroyTexture(metal_two_block);
										SDL_DestroyTexture(metal_three_block);
										SDL_DestroyTexture(metal_hexagon_block);
										SDL_DestroyTexture(metal_polygon1_block);
										SDL_DestroyTexture(metal_trapezoid_block);
										SDL_DestroyTexture(pre_metal_one_block);
										SDL_DestroyTexture(pre_metal_two_block);
										SDL_DestroyTexture(pre_metal_three_block);
										SDL_DestroyTexture(pre_metal_hexagon_block);
										SDL_DestroyTexture(pre_metal_polygon1_block);
										SDL_DestroyTexture(pre_metal_trapezoid_block);
										SDL_DestroyTexture(rock_one_block);
										SDL_DestroyTexture(rock_two_block);
										SDL_DestroyTexture(rock_three_block);
										SDL_DestroyTexture(rock_hexagon_block);
										SDL_DestroyTexture(rock_polygon1_block);
										SDL_DestroyTexture(rock_trapezoid_block);
										SDL_DestroyTexture(pre_rock_one_block);
										SDL_DestroyTexture(pre_rock_two_block);
										SDL_DestroyTexture(pre_rock_three_block);
										SDL_DestroyTexture(pre_rock_hexagon_block);
										SDL_DestroyTexture(pre_rock_polygon1_block);
										SDL_DestroyTexture(pre_rock_trapezoid_block);
										SDL_DestroyTexture(explosion1);
										SDL_DestroyTexture(explosion2);
										SDL_DestroyTexture(explosion3);
										SDL_DestroyTexture(explosion4);
										SDL_DestroyTexture(explosion5);
										SDL_DestroyTexture(explosion6);
										SDL_DestroyTexture(explosion7);
										SDL_DestroyTexture(explosion8);
										SDL_DestroyTexture(explosion9);
										SDL_DestroyTexture(explosion11);
										SDL_DestroyTexture(explosion10);
										SDL_DestroyTexture(explosion12);
										SDL_DestroyTexture(explosion13);
										SDL_DestroyTexture(explosion14);
										SDL_DestroyTexture(explosion15);
										SDL_DestroyTexture(explosion16);
										SDL_DestroyTexture(explosion17);
										SDL_DestroyTexture(bombimg);
										SDL_DestroyTexture(cat);
										SDL_DestroyTexture(stone);
										SDL_DestroyTexture(large_soil);
										SDL_DestroyTexture(ground_cone_small);
										SDL_DestroyTexture(hill);
										SDL_DestroyTexture(line);
										SDL_DestroyTexture(star);
										SDL_DestroyTexture(blood_dot);
										SDL_DestroyTexture(stop_retry);
										SDL_DestroyTexture(pause);
										SDL_DestroyTexture(normal_speed);
										SDL_DestroyTexture(x2speed);
										SDL_DestroyTexture(speed_select);
										SDL_DestroyTexture(running_end_menu);
										SDL_DestroyTexture(back_to_home);
										SDL_DestroyTexture(next_arrow);
										SDL_DestroyTexture(re_arrow);
										if (population != 0) {
											for (i = 0; i < population; i++) {
												for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
												free(object[i].shape->vertex); free(object[i].shape->normal);
												free(object[i].shape->vertex_contact_index);
												free(object[i].shape->vertex_contact);
												free(object[i].shape);
											}
											free(object);
										}
										if (population != 0) population = 0;
										free(contact);
										if (bomb_population != 0) free(bomb);
										bomb_population = 0;
										retry = true;
										Mix_HaltChannel(-1);
										Mix_PlayChannel(-1, click_sound, 0);
									}
								}
								break;
							}
							}
						}
					}

					if (cat_hurt == true) time_ = endtime + 1.0f;

					if (time_ >= endtime) {
						if (end == false) {
							for (i = 0; i < population; i++) {
								for (j = 0; j < object[i].shape->vertex_num; j++) {
									if (object[i].x[1] + object[i].shape->vertex[j][1] <= max_height) max_height = object[i].x[1] + object[i].shape->vertex[j][1];
								}
							}
							if (cat_hurt == true) {
								clear = false;
								cat_framestart = SDL_GetPerformanceCounter();
								for (i = 0; i < 10; i++) {
									cat_x[i] = cat_hurt_coordinate[0];
									cat_y[i] = cat_hurt_coordinate[1];
								}
							}
							if (max_height <= height/trans && cat_hurt == false) {
								clear = true;
								if (save.save_stage == stage_number - 1 && save.save_step == step_number - 1) {
									if (save.save_step == 7) { save.save_stage = save.save_stage + 1; save.save_step = 0; }
									else save.save_step = save.save_step + 1;
								}
								if (save.record[stage_number - 1][step_number - 1][0] < 1360 - max_height) save.record[stage_number - 1][step_number - 1][0] = 1360 - max_height;
								if (save.record[stage_number - 1][step_number - 1][1] > initial_budget - budget)save.record[stage_number - 1][step_number - 1][1] = initial_budget - budget;
								Mix_PlayChannel(-1, fanfare, 0);
							}
							savefile = fopen("save.sav", "wb");
							fwrite(&save, sizeof(save), 1, savefile);
							fclose(savefile);
							star_framestart = SDL_GetPerformanceCounter();
							end = true;
						}

						if (clear == true) {
							if (star_accumulate < 3.0) {
								star_currenttime = SDL_GetPerformanceCounter();
								star_deltime = (float)(star_framestart - star_currenttime) / ((float)SDL_GetPerformanceFrequency());
								star_accumulate = star_accumulate + star_deltime;
								star_framestart = SDL_GetPerformanceCounter();
								for (i = 0; i < 10; i++) {
									star_x[i] = star_x[i] + star_vx[i] *star_deltime;
									star_vy[i] = star_vy[i] + 10 * star_deltime;
									star_y[i] = star_y[i] + star_vy[i] *star_deltime;
									star_th[i] = star_th[i] + star_w[i] *star_deltime;
								}
							}
						}

						if (cat_hurt == true) {
							if (cat_accumulate < 3.0) {
								cat_currenttime = SDL_GetPerformanceCounter();
								cat_deltime = (float)(cat_framestart - cat_currenttime) / ((float)SDL_GetPerformanceFrequency());
								cat_accumulate = cat_accumulate + cat_deltime;
								cat_framestart = SDL_GetPerformanceCounter();
								for (i = 0; i < 10; i++) {
									cat_x[i] = cat_x[i] + cat_vx[i] * cat_deltime;
									cat_vy[i] = cat_vy[i] + 10 * cat_deltime;
									cat_y[i] = cat_y[i] + cat_vy[i] * cat_deltime;
									cat_th[i] = cat_th[i] + cat_w[i] * cat_deltime;
								}
							}
						}

						while (SDL_PollEvent(&event)) {
							if (event.type == SDL_QUIT) quit = true;
							if (event.type == SDL_MOUSEBUTTONDOWN) {
								if (event.button.button == SDL_BUTTON_LEFT) {
									int x, y;
									x = event.motion.x;
									y = event.motion.y;
									if (x >= 289 && x < 410 && y >= 477 && y < 543) { // click back to stage select button
										stage_start = false;
										SDL_DestroyTexture(map);
										SDL_DestroyTexture(right_menu);
										SDL_DestroyTexture(arrow);
										SDL_DestroyTexture(wood_one_block);
										SDL_DestroyTexture(wood_two_block);
										SDL_DestroyTexture(wood_three_block);
										SDL_DestroyTexture(wood_hexagon_block);
										SDL_DestroyTexture(wood_polygon1_block);
										SDL_DestroyTexture(wood_trapezoid_block);
										SDL_DestroyTexture(pre_wood_one_block);
										SDL_DestroyTexture(pre_wood_two_block);
										SDL_DestroyTexture(pre_wood_three_block);
										SDL_DestroyTexture(pre_wood_hexagon_block);
										SDL_DestroyTexture(pre_wood_polygon1_block);
										SDL_DestroyTexture(pre_wood_trapezoid_block);
										SDL_DestroyTexture(metal_one_block);
										SDL_DestroyTexture(metal_two_block);
										SDL_DestroyTexture(metal_three_block);
										SDL_DestroyTexture(metal_hexagon_block);
										SDL_DestroyTexture(metal_polygon1_block);
										SDL_DestroyTexture(metal_trapezoid_block);
										SDL_DestroyTexture(pre_metal_one_block);
										SDL_DestroyTexture(pre_metal_two_block);
										SDL_DestroyTexture(pre_metal_three_block);
										SDL_DestroyTexture(pre_metal_hexagon_block);
										SDL_DestroyTexture(pre_metal_polygon1_block);
										SDL_DestroyTexture(pre_metal_trapezoid_block);
										SDL_DestroyTexture(rock_one_block);
										SDL_DestroyTexture(rock_two_block);
										SDL_DestroyTexture(rock_three_block);
										SDL_DestroyTexture(rock_hexagon_block);
										SDL_DestroyTexture(rock_polygon1_block);
										SDL_DestroyTexture(rock_trapezoid_block);
										SDL_DestroyTexture(pre_rock_one_block);
										SDL_DestroyTexture(pre_rock_two_block);
										SDL_DestroyTexture(pre_rock_three_block);
										SDL_DestroyTexture(pre_rock_hexagon_block);
										SDL_DestroyTexture(pre_rock_polygon1_block);
										SDL_DestroyTexture(pre_rock_trapezoid_block);
										SDL_DestroyTexture(explosion1);
										SDL_DestroyTexture(explosion2);
										SDL_DestroyTexture(explosion3);
										SDL_DestroyTexture(explosion4);
										SDL_DestroyTexture(explosion5);
										SDL_DestroyTexture(explosion6);
										SDL_DestroyTexture(explosion7);
										SDL_DestroyTexture(explosion8);
										SDL_DestroyTexture(explosion9);
										SDL_DestroyTexture(explosion11);
										SDL_DestroyTexture(explosion10);
										SDL_DestroyTexture(explosion12);
										SDL_DestroyTexture(explosion13);
										SDL_DestroyTexture(explosion14);
										SDL_DestroyTexture(explosion15);
										SDL_DestroyTexture(explosion16);
										SDL_DestroyTexture(explosion17);
										SDL_DestroyTexture(bombimg);
										SDL_DestroyTexture(cat);
										SDL_DestroyTexture(stone);
										SDL_DestroyTexture(large_soil);
										SDL_DestroyTexture(ground_cone_small);
										SDL_DestroyTexture(hill);
										SDL_DestroyTexture(line);
										SDL_DestroyTexture(star);
										SDL_DestroyTexture(blood_dot);
										SDL_DestroyTexture(stop_retry);
										SDL_DestroyTexture(pause);
										SDL_DestroyTexture(normal_speed);
										SDL_DestroyTexture(x2speed);
										SDL_DestroyTexture(speed_select);
										SDL_DestroyTexture(running_end_menu);
										SDL_DestroyTexture(back_to_home);
										SDL_DestroyTexture(next_arrow);
										SDL_DestroyTexture(re_arrow);
										if (stage_number == 1)scene = stage_select1;
										else if (stage_number == 2)scene = stage_select2;
										else if (stage_number == 3)scene = stage_select3;
										else if (stage_number == 4)scene = stage_select4;
										else if (stage_number == 5)scene = stage_select5;
										else if (stage_number == 6)scene = stage_select6;
										if (population != 0) {
											for (i = 0; i < population; i++) {
												for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
												free(object[i].shape->vertex); free(object[i].shape->normal);
												free(object[i].shape->vertex_contact_index);
												free(object[i].shape->vertex_contact);
												free(object[i].shape);
											}
											free(object);
										}
										free(contact);
										if (population != 0) {
											for (i = 0; i < population; i++) {
												for (j = 0; j < pre_object[i].shape->vertex_num; j++) { free(pre_object[i].shape->vertex[j]); free(pre_object[i].shape->normal[j]); }
												free(pre_object[i].shape->vertex); free(pre_object[i].shape->normal);
												free(pre_object[i].shape->vertex_contact_index);
												free(pre_object[i].shape->vertex_contact);
												free(pre_object[i].shape);
											}
											free(pre_object);
										}
										if (population != 0) population = 0;
										if (bomb_population != 0) free(bomb);
										bomb_population = 0;
										retry = false;
										stage_to_select_menu = true;
										Mix_PlayChannel(-1, click_sound, 0);
										Mix_HaltMusic();
										Mix_FadeInMusic(main_music, -1, 1000);
									}

									else if (x >= 513 && x < 580 && y >= 472 && y < 546) { // click retry button
										stage_start = false;
										stage_prepare = true;
										SDL_DestroyTexture(map);
										SDL_DestroyTexture(right_menu);
										SDL_DestroyTexture(arrow);
										SDL_DestroyTexture(wood_one_block);
										SDL_DestroyTexture(wood_two_block);
										SDL_DestroyTexture(wood_three_block);
										SDL_DestroyTexture(wood_hexagon_block);
										SDL_DestroyTexture(wood_polygon1_block);
										SDL_DestroyTexture(wood_trapezoid_block);
										SDL_DestroyTexture(pre_wood_one_block);
										SDL_DestroyTexture(pre_wood_two_block);
										SDL_DestroyTexture(pre_wood_three_block);
										SDL_DestroyTexture(pre_wood_hexagon_block);
										SDL_DestroyTexture(pre_wood_polygon1_block);
										SDL_DestroyTexture(pre_wood_trapezoid_block);
										SDL_DestroyTexture(metal_one_block);
										SDL_DestroyTexture(metal_two_block);
										SDL_DestroyTexture(metal_three_block);
										SDL_DestroyTexture(metal_hexagon_block);
										SDL_DestroyTexture(metal_polygon1_block);
										SDL_DestroyTexture(metal_trapezoid_block);
										SDL_DestroyTexture(pre_metal_one_block);
										SDL_DestroyTexture(pre_metal_two_block);
										SDL_DestroyTexture(pre_metal_three_block);
										SDL_DestroyTexture(pre_metal_hexagon_block);
										SDL_DestroyTexture(pre_metal_polygon1_block);
										SDL_DestroyTexture(pre_metal_trapezoid_block);
										SDL_DestroyTexture(rock_one_block);
										SDL_DestroyTexture(rock_two_block);
										SDL_DestroyTexture(rock_three_block);
										SDL_DestroyTexture(rock_hexagon_block);
										SDL_DestroyTexture(rock_polygon1_block);
										SDL_DestroyTexture(rock_trapezoid_block);
										SDL_DestroyTexture(pre_rock_one_block);
										SDL_DestroyTexture(pre_rock_two_block);
										SDL_DestroyTexture(pre_rock_three_block);
										SDL_DestroyTexture(pre_rock_hexagon_block);
										SDL_DestroyTexture(pre_rock_polygon1_block);
										SDL_DestroyTexture(pre_rock_trapezoid_block);
										SDL_DestroyTexture(explosion1);
										SDL_DestroyTexture(explosion2);
										SDL_DestroyTexture(explosion3);
										SDL_DestroyTexture(explosion4);
										SDL_DestroyTexture(explosion5);
										SDL_DestroyTexture(explosion6);
										SDL_DestroyTexture(explosion7);
										SDL_DestroyTexture(explosion8);
										SDL_DestroyTexture(explosion9);
										SDL_DestroyTexture(explosion11);
										SDL_DestroyTexture(explosion10);
										SDL_DestroyTexture(explosion12);
										SDL_DestroyTexture(explosion13);
										SDL_DestroyTexture(explosion14);
										SDL_DestroyTexture(explosion15);
										SDL_DestroyTexture(explosion16);
										SDL_DestroyTexture(explosion17);
										SDL_DestroyTexture(bombimg);
										SDL_DestroyTexture(cat);
										SDL_DestroyTexture(stone);
										SDL_DestroyTexture(large_soil);
										SDL_DestroyTexture(ground_cone_small);
										SDL_DestroyTexture(hill);
										SDL_DestroyTexture(line);
										SDL_DestroyTexture(star);
										SDL_DestroyTexture(blood_dot);
										SDL_DestroyTexture(stop_retry);
										SDL_DestroyTexture(pause);
										SDL_DestroyTexture(normal_speed);
										SDL_DestroyTexture(x2speed);
										SDL_DestroyTexture(speed_select);
										SDL_DestroyTexture(running_end_menu);
										SDL_DestroyTexture(back_to_home);
										SDL_DestroyTexture(next_arrow);
										SDL_DestroyTexture(re_arrow);
										if (population != 0) {
											for (i = 0; i < population; i++) {
												for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
												free(object[i].shape->vertex); free(object[i].shape->normal);
												free(object[i].shape->vertex_contact_index);
												free(object[i].shape->vertex_contact);
												free(object[i].shape);
											}
											free(object);
										}
										if (population != 0) population = 0;
										free(contact);
										if (bomb_population != 0) free(bomb);
										bomb_population = 0;
										retry = true;
										Mix_PlayChannel(-1, click_sound, 0);
									}
									else if (x>=688 && x<779 && y>= 477 && y<841 && clear == true && !(stage_number==6 && step_number==8)) { // click next button
										stage_start = false;
										stage_prepare = true;
										SDL_DestroyTexture(map);
										SDL_DestroyTexture(right_menu);
										SDL_DestroyTexture(arrow);
										SDL_DestroyTexture(wood_one_block);
										SDL_DestroyTexture(wood_two_block);
										SDL_DestroyTexture(wood_three_block);
										SDL_DestroyTexture(wood_hexagon_block);
										SDL_DestroyTexture(wood_polygon1_block);
										SDL_DestroyTexture(wood_trapezoid_block);
										SDL_DestroyTexture(pre_wood_one_block);
										SDL_DestroyTexture(pre_wood_two_block);
										SDL_DestroyTexture(pre_wood_three_block);
										SDL_DestroyTexture(pre_wood_hexagon_block);
										SDL_DestroyTexture(pre_wood_polygon1_block);
										SDL_DestroyTexture(pre_wood_trapezoid_block);
										SDL_DestroyTexture(metal_one_block);
										SDL_DestroyTexture(metal_two_block);
										SDL_DestroyTexture(metal_three_block);
										SDL_DestroyTexture(metal_hexagon_block);
										SDL_DestroyTexture(metal_polygon1_block);
										SDL_DestroyTexture(metal_trapezoid_block);
										SDL_DestroyTexture(pre_metal_one_block);
										SDL_DestroyTexture(pre_metal_two_block);
										SDL_DestroyTexture(pre_metal_three_block);
										SDL_DestroyTexture(pre_metal_hexagon_block);
										SDL_DestroyTexture(pre_metal_polygon1_block);
										SDL_DestroyTexture(pre_metal_trapezoid_block);
										SDL_DestroyTexture(rock_one_block);
										SDL_DestroyTexture(rock_two_block);
										SDL_DestroyTexture(rock_three_block);
										SDL_DestroyTexture(rock_hexagon_block);
										SDL_DestroyTexture(rock_polygon1_block);
										SDL_DestroyTexture(rock_trapezoid_block);
										SDL_DestroyTexture(pre_rock_one_block);
										SDL_DestroyTexture(pre_rock_two_block);
										SDL_DestroyTexture(pre_rock_three_block);
										SDL_DestroyTexture(pre_rock_hexagon_block);
										SDL_DestroyTexture(pre_rock_polygon1_block);
										SDL_DestroyTexture(pre_rock_trapezoid_block);
										SDL_DestroyTexture(explosion1);
										SDL_DestroyTexture(explosion2);
										SDL_DestroyTexture(explosion3);
										SDL_DestroyTexture(explosion4);
										SDL_DestroyTexture(explosion5);
										SDL_DestroyTexture(explosion6);
										SDL_DestroyTexture(explosion7);
										SDL_DestroyTexture(explosion8);
										SDL_DestroyTexture(explosion9);
										SDL_DestroyTexture(explosion11);
										SDL_DestroyTexture(explosion10);
										SDL_DestroyTexture(explosion12);
										SDL_DestroyTexture(explosion13);
										SDL_DestroyTexture(explosion14);
										SDL_DestroyTexture(explosion15);
										SDL_DestroyTexture(explosion16);
										SDL_DestroyTexture(explosion17);
										SDL_DestroyTexture(bombimg);
										SDL_DestroyTexture(cat);
										SDL_DestroyTexture(stone);
										SDL_DestroyTexture(large_soil);
										SDL_DestroyTexture(ground_cone_small);
										SDL_DestroyTexture(hill);
										SDL_DestroyTexture(line);
										SDL_DestroyTexture(star);
										SDL_DestroyTexture(blood_dot);
										SDL_DestroyTexture(stop_retry);
										SDL_DestroyTexture(pause);
										SDL_DestroyTexture(normal_speed);
										SDL_DestroyTexture(x2speed);
										SDL_DestroyTexture(speed_select);
										SDL_DestroyTexture(running_end_menu);
										SDL_DestroyTexture(back_to_home);
										SDL_DestroyTexture(next_arrow);
										SDL_DestroyTexture(re_arrow);
										if (population != 0) {
											for (i = 0; i < population; i++) {
												for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
												free(object[i].shape->vertex); free(object[i].shape->normal);
												free(object[i].shape->vertex_contact_index);
												free(object[i].shape->vertex_contact);
												free(object[i].shape);
											}
											free(object);
										}
										if (population != 0) population = 0;
										free(contact);
										if (bomb_population != 0) free(bomb);
										bomb_population = 0;
										if (step_number == 8) {
											stage_number = stage_number + 1;
											step_number = 1;
										}
										else step_number = step_number + 1;
										next_click = true;
										Mix_PlayChannel(-1, click_sound, 0);
									}
									else if (x > 1117 && x < 1160 && y>688 && y < 738) speed = 0;
									else if (x > 1186 && x < 1234 && y>688 && y < 739) speed = 1;
									else if (x > 1255 && x < 1347 && y>688 && y < 739) speed = 2;
								}
							}
						}

					}

/*					for (i = 0; i < population; i++) for (j = 0; j < object[i].shape->vertex_num; j++) {
						float vertex[2];
						vertex[0] = object[i].x[0] + object[i].shape->vertex[j][0];
						vertex[1] = object[i].x[1] + object[i].shape->vertex[j][1];
						stretchTextureEx(renderer, vertex[0]*trans - 2, vertex[1]*trans - 2, 5, 5, reddot, 0);
					}*/

					drawTexture(renderer, 0, 0, map);
					drawTexture(renderer, 0, height, line);
					drawTexture(renderer, 1004, 0, right_menu);
					if (speed == 0) drawTexture(renderer, 1090, 664, speed_select);
					else if (speed == 1)drawTexture(renderer, 1162, 664, speed_select);
					else if (speed == 2) drawTexture(renderer, 1253, 664, speed_select);
					drawTexture(renderer, 1046, 685, stop_retry);
					drawTexture(renderer, 1117, 688, pause);
					drawTexture(renderer, 1186, 688, normal_speed);
					drawTexture(renderer, 1255, 688, x2speed);

					for (i = 0; i < population; i++) {
						switch (object[i].shape->number) {
						case 1:
							if (object[i].material.number == 2 || object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, wood_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3 || object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, metal_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1 || object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, rock_one_block, object[i].th * 180 / pi);
							break;
						case 2:
							if (object[i].material.number == 2 || object[i].material.number == 5) stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, wood_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3 || object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, metal_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1 || object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, rock_two_block, object[i].th * 180 / pi);
							break;
						case 3:
							if (object[i].material.number == 2 || object[i].material.number == 5) stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, wood_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3 || object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, metal_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1 || object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, rock_three_block, object[i].th * 180 / pi);
							break;
						case 6:
							if (object[i].material.number == 2 || object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), wood_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3 || object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), metal_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1 || object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), rock_trapezoid_block, object[i].th * 180 / pi);
							break;
						case 8:
							if (object[i].material.number == 2 || object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, wood_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3 || object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, metal_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1 || object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, rock_hexagon_block, object[i].th * 180 / pi);
							break;
						case 9:
							if (object[i].material.number == 2 || object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, wood_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3 || object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, metal_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1 || object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, rock_polygon1_block, object[i].th * 180 / pi);
							break;
						case 80:
							stretchTextureEx(renderer, object[i].x[0] * trans - 49.96*Length*trans / 40.0, object[i].x[1] * trans - 33.02*Length*trans / 40.0, 101 * Length*trans / 40.0, 69.0*Length*trans / 40.0, stone, object[i].th * 180 / pi);
							break;
						case 81:
							stretchTextureEx_revise(renderer, object[i].x[0] * trans - 25.5*Length*trans / 40.0 - 3, object[i].x[1] * trans - 31.0*Length*trans / 40.0, 51 * Length*trans / 40.0, 62.0*Length*trans / 40.0, 0.733511*Length*trans, 0.759908*Length*trans, cat, object[i].th * 180 / pi);
							break;
						case 97:
							stretchTextureEx_revise(renderer, object[i].x[0] * trans - 2.946671f*Length*trans, object[i].x[1] * trans - 3.543418f*Length*trans, 250.0*Length*trans / 40.0, 279.0*Length*trans / 40.0, 117.809f*Length*trans / 40.0, 142.2465f*Length*trans / 40.0f, hill, object[i].th * 180 / pi);
							break;
						case 98:
							stretchTextureEx_revise(renderer, object[i].x[0] * trans - 77.5*Length*trans / 40.0, object[i].x[1] * trans - 4459.0*Length*trans / (219.0*40.0), 155.0*Length*trans / 40.0, 58.0*Length*trans / 40.0, 77.5*Length*trans / 40.0, 20.36*Length*trans / 40.0, ground_cone_small, object[i].th * 180 / pi);
							break;
						case 99:
							stretchTextureEx(renderer, object[i].x[0] * trans - 7 * Length*trans, object[i].x[1] * trans - 2 * Length*trans, 14 * Length*trans, 4 * Length*trans, large_soil, object[i].th * 180 / pi);
							break;
						}
					}

					if (time_ >= endtime) {

						drawTexture(renderer, 230, 207, running_end_menu);
						drawTexture(renderer, 289, 477, back_to_home);
						drawTexture(renderer, 513, 472, re_arrow);
						if (clear == true && !(stage_number == 6 && step_number == 8))drawTexture(renderer, 688, 477, next_arrow);

						if (clear == true) {
							sprintf(text_string, "Clear");
							for (i = 0; i < 10; i++) stretchTextureEx(renderer, star_x[i] * trans, star_y[i] * trans, 37, 32, star, star_th[i] * 180 / pi);
						}
						else sprintf(text_string, "  Fail");
						text_surface = TTF_RenderText_Blended(font70, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 454, 222, text);
						drawTexture(renderer, 454, 222, text);
						drawTexture(renderer, 454, 222, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);

						if(cat_hurt==true) for (i = 0; i < 10; i++) stretchTextureEx(renderer, cat_x[i] * trans, cat_y[i] * trans, 10, 10, blood_dot, cat_th[i] * 180 / pi);

						if (1360 - max_height >= 0) sprintf(text_string, "final height  %.1fm", (1360 - max_height)*0.01);
						else sprintf(text_string, "final height  x");
						text_surface = TTF_RenderText_Blended(font32, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 416, 298, text);
						drawTexture(renderer, 416, 298, text);
						drawTexture(renderer, 416, 298, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);

						sprintf(text_string, "used budget  %d$", initial_budget - budget);
						text_surface = TTF_RenderText_Blended(font32, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 414, 341, text);
						drawTexture(renderer, 414, 341, text);
						drawTexture(renderer, 414, 341, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);

						if (save.record[stage_number - 1][step_number - 1][0] >= 0) sprintf(text_string, "your highest final height  %.1fm", save.record[stage_number - 1][step_number - 1][0] * 0.01);
						else sprintf(text_string, "your highest final height  x");
						text_surface = TTF_RenderText_Blended(font32, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 311, 381, text);
						drawTexture(renderer, 311, 381, text);
						drawTexture(renderer, 311, 381, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);

						if (save.record[stage_number - 1][step_number - 1][1] != FLT_MAX) sprintf(text_string, "your lowest used budget  %.0f$", save.record[stage_number - 1][step_number - 1][1]);
						else sprintf(text_string, "your lowest used budget  x");
						text_surface = TTF_RenderText_Blended(font32, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 323, 421, text);
						drawTexture(renderer, 323, 421, text);
						drawTexture(renderer, 323, 421, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);
					}

					if (time_<bomb_animation_time*17) {
						for (i = 0; i < bomb_population; i++) {
							if (time_ < bomb_animation_time) { stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0); stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion1, 0); }
									else if(time_<bomb_animation_time*2){ stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0); stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion2, 0); }
									else if (time_ < bomb_animation_time*3) { stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0); stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion3, 0); }
									else if (time_ < bomb_animation_time*4) { stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0); stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion4, 0); }
									else if (time_ < bomb_animation_time*5) { stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0); stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion5, 0); }
									else if (time_<bomb_animation_time*6) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion6, 0);
									else if (time_<bomb_animation_time*7) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion7, 0);
									else if (time_<bomb_animation_time*8) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion8, 0);
									else if (time_<bomb_animation_time*9) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion9, 0);
									else if (time_<bomb_animation_time*10) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion10, 0);
									else if (time_<bomb_animation_time*11) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion11, 0);
									else if (time_<bomb_animation_time*12) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion12, 0);
									else if (time_<bomb_animation_time*13) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion13, 0);
									else if (time_<bomb_animation_time*14) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion14, 0);
									else if (time_<bomb_animation_time*15) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion15, 0);
									else if (time_<bomb_animation_time*16) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion16, 0);
									else if (time_<bomb_animation_time*17) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion17, 0);
						}
					}

					if(retry == false && stage_to_select_menu==false && next_click == false) SDL_RenderPresent(renderer);

					total_frame_end = SDL_GetPerformanceCounter();
					total_time = (float)(total_frame_end - total_frame_start) / ((float)SDL_GetPerformanceFrequency());
					total_delay_time = (int)(((1 / fpss)*rendering_factor - total_time) * 1000);

					if(total_delay_time>0) SDL_Delay(total_delay_time);
					else SDL_Delay(1);

					if (!(scene == stage_select1 || scene == stage_select2 || scene == stage_select3 || scene == stage_select4 || scene == stage_select5 || scene == stage_select6) && !(retry == true) && !(next_click == true)) SDL_RenderClear(renderer);
				}
			}
		}
		else if (scene == map_edit) {

			time_t timer_;
			struct tm *tp;
			int save_year;
			int save_month;
			int save_day;
			int save_hour;
			int save_min;
			int save_sec;

			int map_number = 0;

			int i, j, k;
			char text_string[40];

			Object * object = NULL;
			Object * tempt_object;
			Bomb * bomb = NULL;
			Bomb * tempt_bomb;
			int bomb_population = 0;
			int population = 0;

			int material = 5; // pre_wood
			int mouse_x, mouse_y;
			bool back_to_map_button = false;
			int click_block = -1;
			int click_bomb = -1;
			int click_x, click_y;

			bool click_one_block = false;
			bool click_two_block = false;
			bool click_three_block = false;
			bool click_hexagon_block = false;
			bool click_polygon1_block = false;
			bool click_trapezoid_block = false;
			bool click_line = false;
			bool copy = false;

			float angle = 0;
			float height = 250.0f;
			float frequency = 0.1f;
			float time_duration = 1.5f;
			float amplitude = 0.1f;
			int earthquake_type = 0;
			int budget = 0;
			int hundreds = 0; int tenth = 0; int units = 0;

			float underbar_time = 0;
			bool cancle_mouse = false;
			bool save_mouse = false;

			bool save_screen = false;
			char * map_name;
			map_name = (char *)malloc(sizeof(char) * 9);
			map_name[0] = 'y'; map_name[1] = 'o'; map_name[2] = 'u'; map_name[3] = 'r'; map_name[4] = ' '; map_name[5] = 'm'; map_name[6] = 'a'; map_name[7] = 'p'; map_name[8] = '\0';
			int map_name_length = 9;

			int map1_name_length = 0;
			char * map1_name;
			int map2_name_length = 0;
			char * map2_name;
			int map3_name_length = 0;
			char * map3_name;

			int map1_year, map2_year, map3_year;
			int map1_month, map2_month, map3_month;
			int map1_day, map2_day, map3_day;
			int map1_hour, map2_hour, map3_hour;
			int map1_min, map2_min, map3_min;
			int map1_sec, map2_sec, map3_sec;

			if (map_editor_texture == false) {
				map_editor_basic = loadTexture("resource\\map_editor_basic.png");
				map_editor_menu = loadTexture("resource\\map_editor_menu.png");
				trash_bin = loadTexture("resource\\trash_bin.png");
				write = loadTexture("resource\\write.png");
				play = loadTexture("resource\\play.png");
				next = loadTexture("resource\\next.png");
				mainmenu_button = loadTexture("resource\\mainmenu.png");
				arrow = loadTexture("resource\\arrow.png");
				map_editor_texture = true;
			}

			bool map1load;
			bool map2load;
			bool map3load;
			FILE * map1;
			FILE * map2;
			FILE * map3;

			bool map1_delete = false;
			bool map2_delete = false;
			bool map3_delete = false;

			bool map_load = false;

			map1 = fopen("map1.dat", "rb");
			if (map1 == NULL) map1load = false;
			else {
				map1load = true;
				fread(&map1_name_length, sizeof(map1_name_length), 1, map1);
				map1_name = (char *)malloc(sizeof(char)*map1_name_length);
				for (i = 0; i < map1_name_length;i++ ) fread(&map1_name[i], sizeof(map1_name[i]), 1, map1);
				fread(&map1_year, sizeof(map1_year), 1, map1);
				fread(&map1_month, sizeof(map1_month), 1, map1);
				fread(&map1_day, sizeof(map1_day), 1, map1);
				fread(&map1_hour, sizeof(map1_hour), 1, map1);
				fread(&map1_min, sizeof(map1_min), 1, map1);
				fread(&map1_sec, sizeof(map1_sec), 1, map1);
				fclose(map1);
			}
			map2 = fopen("map2.dat", "rb");
			if (map2 == NULL) map2load = false;
			else {
				map2load = true;
				fread(&map2_name_length, sizeof(map2_name_length), 1, map2);
				map2_name = (char *)malloc(sizeof(char)*map2_name_length);
				for (i = 0; i < map2_name_length; i++) fread(&map2_name[i], sizeof(map2_name[i]), 1, map2);
				fread(&map2_year, sizeof(map2_year), 1, map2);
				fread(&map2_month, sizeof(map2_month), 1, map2);
				fread(&map2_day, sizeof(map2_day), 1, map2);
				fread(&map2_hour, sizeof(map2_hour), 1, map2);
				fread(&map2_min, sizeof(map2_min), 1, map2);
				fread(&map2_sec, sizeof(map2_sec), 1, map2);
				fclose(map2);
			}
			map3 = fopen("map3.dat", "rb");
			if (map3 == NULL) map3load = false;
			else {
				map3load = true;
				fread(&map3_name_length, sizeof(map3_name_length), 1, map3);
				map3_name = (char *)malloc(sizeof(char)*map3_name_length);
				for (i = 0; i < map3_name_length; i++) fread(&map3_name[i], sizeof(map3_name[i]), 1, map3);
				fread(&map3_year, sizeof(map3_year), 1, map3);
				fread(&map3_month, sizeof(map3_month), 1, map3);
				fread(&map3_day, sizeof(map3_day), 1, map3);
				fread(&map3_hour, sizeof(map3_hour), 1, map3);
				fread(&map3_min, sizeof(map3_min), 1, map3);
				fread(&map3_sec, sizeof(map3_sec), 1, map3);
				fclose(map3);
			}

			bool map_editing = false;
			bool map_running = false;

			bool map1yes_button = false;
			bool map2yes_button = false;
			bool map3yes_button = false;
			bool map1no_button = false;
			bool map2no_button = false;
			bool map3no_button = false;

			bool map1trash_button = false;
			bool map1write_button = false;
			bool map1play_button = false;
			bool map2trash_button = false;
			bool map2write_button = false;
			bool map2play_button = false;
			bool map3trash_button = false;
			bool map3write_button = false;
			bool map3play_button = false;

			bool map_editor_save_button = false;

			while (quit == false && scene == map_edit && map_editing == false && map_running == false) {
				total_frame_start = SDL_GetPerformanceCounter();
				while (SDL_PollEvent(&event)) {
					switch (event.type) {
					case SDL_QUIT:
						quit = true;
						break;
					case SDL_MOUSEBUTTONDOWN:
						if (event.button.button == SDL_BUTTON_LEFT) {
							int x, y;
							x = event.motion.x;
							y = event.motion.y;
							if (x >= 1135 && x <= 1293 && y >= 57 && y <= 119) { // click next
								scene = stage_select1;
								stage_number = 1;
								SDL_DestroyTexture(map_editor_basic);
								SDL_DestroyTexture(map_editor_menu);
								SDL_DestroyTexture(trash_bin);
								SDL_DestroyTexture(write);
								SDL_DestroyTexture(play);
								SDL_DestroyTexture(next);
								SDL_DestroyTexture(mainmenu_button);
								SDL_DestroyTexture(arrow);
								if (map1_name_length > 0) free(map1_name);
								if (map2_name_length > 0) free(map2_name);
								if (map3_name_length > 0) free(map3_name);
								map_editor_texture = false;
								escape_map_editor = true;
								Mix_PlayChannel(-1, page, 0);
							}
							else if (x >= 975 && x <= 1322 && y >= 669 && y <= 740) { // click click mainmenu button
								map_editor_texture = false;
								mainmenubutton = false;
								scene = main_menu;
								SDL_DestroyTexture(map_editor_basic);
								SDL_DestroyTexture(map_editor_menu);
								SDL_DestroyTexture(trash_bin);
								SDL_DestroyTexture(write);
								SDL_DestroyTexture(play);
								SDL_DestroyTexture(next);
								SDL_DestroyTexture(mainmenu_button);
								SDL_DestroyTexture(arrow);
								if (map1_name_length > 0) free(map1_name);
								if (map2_name_length > 0) free(map2_name);
								if (map3_name_length > 0) free(map3_name);
								escape_map_editor = true;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 217 && x <= 311 && y >= 539 && y <= 633 && map1load == false) { // map 1 create
								editor_back = loadTexture("resource\\editor_back.png");
								right_menu = loadTexture("resource\\stage_menu.png");
								back_to_stage = loadTexture("resource\\back_to_stage.png");
								pre_wood_one_block = loadTexture("resource\\pre_wood_one.png");
								pre_wood_two_block = loadTexture("resource\\pre_wood_two.png");
								pre_wood_three_block = loadTexture("resource\\pre_wood_three.png");
								pre_wood_hexagon_block = loadTexture("resource\\pre_wood_hexagon.png");
								pre_wood_polygon1_block = loadTexture("resource\\pre_wood_polygon1.png");
								pre_wood_trapezoid_block = loadTexture("resource\\pre_wood_trapezoid.png");
								pre_metal_one_block = loadTexture("resource\\pre_metal_one.png");
								pre_metal_two_block = loadTexture("resource\\pre_metal_two.png");
								pre_metal_three_block = loadTexture("resource\\pre_metal_three.png");
								pre_metal_hexagon_block = loadTexture("resource\\pre_metal_hexagon.png");
								pre_metal_polygon1_block = loadTexture("resource\\pre_metal_polygon1.png");
								pre_metal_trapezoid_block = loadTexture("resource\\pre_metal_trapezoid.png");
								pre_rock_one_block = loadTexture("resource\\pre_rock_one.png");
								pre_rock_two_block = loadTexture("resource\\pre_rock_two.png");
								pre_rock_three_block = loadTexture("resource\\pre_rock_three.png");
								pre_rock_hexagon_block = loadTexture("resource\\pre_rock_hexagon.png");
								pre_rock_polygon1_block = loadTexture("resource\\pre_rock_polygon1.png");
								pre_rock_trapezoid_block = loadTexture("resource\\pre_rock_trapezoid.png");
								bombimg = loadTexture("resource\\bombimg.png");
								cat = loadTexture("resource\\cat.png");
								stone = loadTexture("resource\\stone.png");
								large_soil = loadTexture("resource\\two_soil.png");
								ground_cone_small = loadTexture("resource\\soil_cone_small.png");
								hill = loadTexture("resource\\hill.png");
								line = loadTexture("resource\\line.png");
								velocity_arrow = loadTexture("resource\\velocity_arrow.png");
								running_end_menu = loadTexture("resource\\running_end_menu.png");
								save_map = loadTexture("resource\\save.png");
								back_to_map = loadTexture("resource\\back_to_map.png");
								line_drag = loadTexture("resource\\line_drag.png");
								map_edit_earthquake = loadTexture("resource\\map_edit_earthquake.png");
								small_arrow = loadTexture("resource\\small_arrow.png");
								LR_img = loadTexture("resource\\LR_img.png");
								UD_img = loadTexture("resource\\UD_img.png");
								UDLR_img = loadTexture("resource\\UDLR_img.png");
								rotation_img = loadTexture("resource\\rotation_img.png");
								circle_img = loadTexture("resource\\circle_img.png");
								swing_img = loadTexture("resource\\swing_img.png");
								reddot = loadTexture("resource\\reddot.png");
								input_bar = loadTexture("resource\\input_bar.png");
								map_editing = true;
								map_number = 1;
								Mix_PlayChannel(-1, click_sound, 0);
								Mix_HaltMusic();
								Mix_FadeInMusic(in_stage_music, -1, 1000);
							}
							else if (x >= 636 && x <= 730 && y >= 539 && y <= 633 && map2load == false) { // map 2 create
								editor_back = loadTexture("resource\\editor_back.png");
								right_menu = loadTexture("resource\\stage_menu.png");
								back_to_stage = loadTexture("resource\\back_to_stage.png");
								pre_wood_one_block = loadTexture("resource\\pre_wood_one.png");
								pre_wood_two_block = loadTexture("resource\\pre_wood_two.png");
								pre_wood_three_block = loadTexture("resource\\pre_wood_three.png");
								pre_wood_hexagon_block = loadTexture("resource\\pre_wood_hexagon.png");
								pre_wood_polygon1_block = loadTexture("resource\\pre_wood_polygon1.png");
								pre_wood_trapezoid_block = loadTexture("resource\\pre_wood_trapezoid.png");
								pre_metal_one_block = loadTexture("resource\\pre_metal_one.png");
								pre_metal_two_block = loadTexture("resource\\pre_metal_two.png");
								pre_metal_three_block = loadTexture("resource\\pre_metal_three.png");
								pre_metal_hexagon_block = loadTexture("resource\\pre_metal_hexagon.png");
								pre_metal_polygon1_block = loadTexture("resource\\pre_metal_polygon1.png");
								pre_metal_trapezoid_block = loadTexture("resource\\pre_metal_trapezoid.png");
								pre_rock_one_block = loadTexture("resource\\pre_rock_one.png");
								pre_rock_two_block = loadTexture("resource\\pre_rock_two.png");
								pre_rock_three_block = loadTexture("resource\\pre_rock_three.png");
								pre_rock_hexagon_block = loadTexture("resource\\pre_rock_hexagon.png");
								pre_rock_polygon1_block = loadTexture("resource\\pre_rock_polygon1.png");
								pre_rock_trapezoid_block = loadTexture("resource\\pre_rock_trapezoid.png");
								bombimg = loadTexture("resource\\bombimg.png");
								cat = loadTexture("resource\\cat.png");
								stone = loadTexture("resource\\stone.png");
								large_soil = loadTexture("resource\\two_soil.png");
								ground_cone_small = loadTexture("resource\\soil_cone_small.png");
								hill = loadTexture("resource\\hill.png");
								line = loadTexture("resource\\line.png");
								velocity_arrow = loadTexture("resource\\velocity_arrow.png");
								running_end_menu = loadTexture("resource\\running_end_menu.png");
								save_map = loadTexture("resource\\save.png");
								back_to_map = loadTexture("resource\\back_to_map.png");
								line_drag = loadTexture("resource\\line_drag.png");
								map_edit_earthquake = loadTexture("resource\\map_edit_earthquake.png");
								small_arrow = loadTexture("resource\\small_arrow.png");
								LR_img = loadTexture("resource\\LR_img.png");
								UD_img = loadTexture("resource\\UD_img.png");
								UDLR_img = loadTexture("resource\\UDLR_img.png");
								rotation_img = loadTexture("resource\\rotation_img.png");
								circle_img = loadTexture("resource\\circle_img.png");
								swing_img = loadTexture("resource\\swing_img.png");
								reddot = loadTexture("resource\\reddot.png");
								input_bar = loadTexture("resource\\input_bar.png");
								map_editing = true;
								map_number = 2;
								Mix_PlayChannel(-1, click_sound, 0);
								Mix_HaltMusic();
								Mix_FadeInMusic(in_stage_music, -1, 1000);
							}
							else if (x >= 1055 && x <= 1149 && y >= 539 && y <= 633 && map3load == false) { // map 3 create
								editor_back = loadTexture("resource\\editor_back.png");
								right_menu = loadTexture("resource\\stage_menu.png");
								back_to_stage = loadTexture("resource\\back_to_stage.png");
								pre_wood_one_block = loadTexture("resource\\pre_wood_one.png");
								pre_wood_two_block = loadTexture("resource\\pre_wood_two.png");
								pre_wood_three_block = loadTexture("resource\\pre_wood_three.png");
								pre_wood_hexagon_block = loadTexture("resource\\pre_wood_hexagon.png");
								pre_wood_polygon1_block = loadTexture("resource\\pre_wood_polygon1.png");
								pre_wood_trapezoid_block = loadTexture("resource\\pre_wood_trapezoid.png");
								pre_metal_one_block = loadTexture("resource\\pre_metal_one.png");
								pre_metal_two_block = loadTexture("resource\\pre_metal_two.png");
								pre_metal_three_block = loadTexture("resource\\pre_metal_three.png");
								pre_metal_hexagon_block = loadTexture("resource\\pre_metal_hexagon.png");
								pre_metal_polygon1_block = loadTexture("resource\\pre_metal_polygon1.png");
								pre_metal_trapezoid_block = loadTexture("resource\\pre_metal_trapezoid.png");
								pre_rock_one_block = loadTexture("resource\\pre_rock_one.png");
								pre_rock_two_block = loadTexture("resource\\pre_rock_two.png");
								pre_rock_three_block = loadTexture("resource\\pre_rock_three.png");
								pre_rock_hexagon_block = loadTexture("resource\\pre_rock_hexagon.png");
								pre_rock_polygon1_block = loadTexture("resource\\pre_rock_polygon1.png");
								pre_rock_trapezoid_block = loadTexture("resource\\pre_rock_trapezoid.png");
								bombimg = loadTexture("resource\\bombimg.png");
								cat = loadTexture("resource\\cat.png");
								stone = loadTexture("resource\\stone.png");
								large_soil = loadTexture("resource\\two_soil.png");
								ground_cone_small = loadTexture("resource\\soil_cone_small.png");
								hill = loadTexture("resource\\hill.png");
								line = loadTexture("resource\\line.png");
								velocity_arrow = loadTexture("resource\\velocity_arrow.png");
								running_end_menu = loadTexture("resource\\running_end_menu.png");
								save_map = loadTexture("resource\\save.png");
								back_to_map = loadTexture("resource\\back_to_map.png");
								line_drag = loadTexture("resource\\line_drag.png");
								map_edit_earthquake = loadTexture("resource\\map_edit_earthquake.png");
								small_arrow = loadTexture("resource\\small_arrow.png");
								LR_img = loadTexture("resource\\LR_img.png");
								UD_img = loadTexture("resource\\UD_img.png");
								UDLR_img = loadTexture("resource\\UDLR_img.png");
								rotation_img = loadTexture("resource\\rotation_img.png");
								circle_img = loadTexture("resource\\circle_img.png");
								swing_img = loadTexture("resource\\swing_img.png");
								reddot = loadTexture("resource\\reddot.png");
								input_bar = loadTexture("resource\\input_bar.png");
								map_editing = true;
								map_number = 3;
								Mix_PlayChannel(-1, click_sound, 0);
								Mix_HaltMusic();
								Mix_FadeInMusic(in_stage_music, -1, 1000);
							}
							else if (x >= 216 && x <= 310 && y >= 532 && y <= 626 && map1load == true && map1_delete == false) { // map1 load
								editor_back = loadTexture("resource\\editor_back.png");
								right_menu = loadTexture("resource\\stage_menu.png");
								back_to_stage = loadTexture("resource\\back_to_stage.png");
								pre_wood_one_block = loadTexture("resource\\pre_wood_one.png");
								pre_wood_two_block = loadTexture("resource\\pre_wood_two.png");
								pre_wood_three_block = loadTexture("resource\\pre_wood_three.png");
								pre_wood_hexagon_block = loadTexture("resource\\pre_wood_hexagon.png");
								pre_wood_polygon1_block = loadTexture("resource\\pre_wood_polygon1.png");
								pre_wood_trapezoid_block = loadTexture("resource\\pre_wood_trapezoid.png");
								pre_metal_one_block = loadTexture("resource\\pre_metal_one.png");
								pre_metal_two_block = loadTexture("resource\\pre_metal_two.png");
								pre_metal_three_block = loadTexture("resource\\pre_metal_three.png");
								pre_metal_hexagon_block = loadTexture("resource\\pre_metal_hexagon.png");
								pre_metal_polygon1_block = loadTexture("resource\\pre_metal_polygon1.png");
								pre_metal_trapezoid_block = loadTexture("resource\\pre_metal_trapezoid.png");
								pre_rock_one_block = loadTexture("resource\\pre_rock_one.png");
								pre_rock_two_block = loadTexture("resource\\pre_rock_two.png");
								pre_rock_three_block = loadTexture("resource\\pre_rock_three.png");
								pre_rock_hexagon_block = loadTexture("resource\\pre_rock_hexagon.png");
								pre_rock_polygon1_block = loadTexture("resource\\pre_rock_polygon1.png");
								pre_rock_trapezoid_block = loadTexture("resource\\pre_rock_trapezoid.png");
								bombimg = loadTexture("resource\\bombimg.png");
								cat = loadTexture("resource\\cat.png");
								stone = loadTexture("resource\\stone.png");
								large_soil = loadTexture("resource\\two_soil.png");
								ground_cone_small = loadTexture("resource\\soil_cone_small.png");
								hill = loadTexture("resource\\hill.png");
								line = loadTexture("resource\\line.png");
								velocity_arrow = loadTexture("resource\\velocity_arrow.png");
								running_end_menu = loadTexture("resource\\running_end_menu.png");
								save_map = loadTexture("resource\\save.png");
								back_to_map = loadTexture("resource\\back_to_map.png");
								line_drag = loadTexture("resource\\line_drag.png");
								map_edit_earthquake = loadTexture("resource\\map_edit_earthquake.png");
								small_arrow = loadTexture("resource\\small_arrow.png");
								LR_img = loadTexture("resource\\LR_img.png");
								UD_img = loadTexture("resource\\UD_img.png");
								UDLR_img = loadTexture("resource\\UDLR_img.png");
								rotation_img = loadTexture("resource\\rotation_img.png");
								circle_img = loadTexture("resource\\circle_img.png");
								swing_img = loadTexture("resource\\swing_img.png");
								reddot = loadTexture("resource\\reddot.png");
								input_bar = loadTexture("resource\\input_bar.png");
								map_editing = true;
								map_load = true;
								map_number = 1;
								Mix_PlayChannel(-1, click_sound, 0);
								Mix_HaltMusic();
								Mix_FadeInMusic(in_stage_music, -1, 1000);
							}
							else if (x >= 635 && x <= 729 && y >= 532 && y <= 626 && map2load == true && map2_delete == false) { // map2 load
								editor_back = loadTexture("resource\\editor_back.png");
								right_menu = loadTexture("resource\\stage_menu.png");
								back_to_stage = loadTexture("resource\\back_to_stage.png");
								pre_wood_one_block = loadTexture("resource\\pre_wood_one.png");
								pre_wood_two_block = loadTexture("resource\\pre_wood_two.png");
								pre_wood_three_block = loadTexture("resource\\pre_wood_three.png");
								pre_wood_hexagon_block = loadTexture("resource\\pre_wood_hexagon.png");
								pre_wood_polygon1_block = loadTexture("resource\\pre_wood_polygon1.png");
								pre_wood_trapezoid_block = loadTexture("resource\\pre_wood_trapezoid.png");
								pre_metal_one_block = loadTexture("resource\\pre_metal_one.png");
								pre_metal_two_block = loadTexture("resource\\pre_metal_two.png");
								pre_metal_three_block = loadTexture("resource\\pre_metal_three.png");
								pre_metal_hexagon_block = loadTexture("resource\\pre_metal_hexagon.png");
								pre_metal_polygon1_block = loadTexture("resource\\pre_metal_polygon1.png");
								pre_metal_trapezoid_block = loadTexture("resource\\pre_metal_trapezoid.png");
								pre_rock_one_block = loadTexture("resource\\pre_rock_one.png");
								pre_rock_two_block = loadTexture("resource\\pre_rock_two.png");
								pre_rock_three_block = loadTexture("resource\\pre_rock_three.png");
								pre_rock_hexagon_block = loadTexture("resource\\pre_rock_hexagon.png");
								pre_rock_polygon1_block = loadTexture("resource\\pre_rock_polygon1.png");
								pre_rock_trapezoid_block = loadTexture("resource\\pre_rock_trapezoid.png");
								bombimg = loadTexture("resource\\bombimg.png");
								cat = loadTexture("resource\\cat.png");
								stone = loadTexture("resource\\stone.png");
								large_soil = loadTexture("resource\\two_soil.png");
								ground_cone_small = loadTexture("resource\\soil_cone_small.png");
								hill = loadTexture("resource\\hill.png");
								line = loadTexture("resource\\line.png");
								velocity_arrow = loadTexture("resource\\velocity_arrow.png");
								running_end_menu = loadTexture("resource\\running_end_menu.png");
								save_map = loadTexture("resource\\save.png");
								back_to_map = loadTexture("resource\\back_to_map.png");
								line_drag = loadTexture("resource\\line_drag.png");
								map_edit_earthquake = loadTexture("resource\\map_edit_earthquake.png");
								small_arrow = loadTexture("resource\\small_arrow.png");
								LR_img = loadTexture("resource\\LR_img.png");
								UD_img = loadTexture("resource\\UD_img.png");
								UDLR_img = loadTexture("resource\\UDLR_img.png");
								rotation_img = loadTexture("resource\\rotation_img.png");
								circle_img = loadTexture("resource\\circle_img.png");
								swing_img = loadTexture("resource\\swing_img.png");
								reddot = loadTexture("resource\\reddot.png");
								input_bar = loadTexture("resource\\input_bar.png");
								map_editing = true;
								map_load = true;
								map_number = 2;
								Mix_PlayChannel(-1, click_sound, 0);
								Mix_HaltMusic();
								Mix_FadeInMusic(in_stage_music, -1, 1000);
							}
							else if (x >= 1054 && x <= 1148 && y >= 532 && y <= 626 && map3load == true && map3_delete == false) { // map3 load
								editor_back = loadTexture("resource\\editor_back.png");
								right_menu = loadTexture("resource\\stage_menu.png");
								back_to_stage = loadTexture("resource\\back_to_stage.png");
								pre_wood_one_block = loadTexture("resource\\pre_wood_one.png");
								pre_wood_two_block = loadTexture("resource\\pre_wood_two.png");
								pre_wood_three_block = loadTexture("resource\\pre_wood_three.png");
								pre_wood_hexagon_block = loadTexture("resource\\pre_wood_hexagon.png");
								pre_wood_polygon1_block = loadTexture("resource\\pre_wood_polygon1.png");
								pre_wood_trapezoid_block = loadTexture("resource\\pre_wood_trapezoid.png");
								pre_metal_one_block = loadTexture("resource\\pre_metal_one.png");
								pre_metal_two_block = loadTexture("resource\\pre_metal_two.png");
								pre_metal_three_block = loadTexture("resource\\pre_metal_three.png");
								pre_metal_hexagon_block = loadTexture("resource\\pre_metal_hexagon.png");
								pre_metal_polygon1_block = loadTexture("resource\\pre_metal_polygon1.png");
								pre_metal_trapezoid_block = loadTexture("resource\\pre_metal_trapezoid.png");
								pre_rock_one_block = loadTexture("resource\\pre_rock_one.png");
								pre_rock_two_block = loadTexture("resource\\pre_rock_two.png");
								pre_rock_three_block = loadTexture("resource\\pre_rock_three.png");
								pre_rock_hexagon_block = loadTexture("resource\\pre_rock_hexagon.png");
								pre_rock_polygon1_block = loadTexture("resource\\pre_rock_polygon1.png");
								pre_rock_trapezoid_block = loadTexture("resource\\pre_rock_trapezoid.png");
								bombimg = loadTexture("resource\\bombimg.png");
								cat = loadTexture("resource\\cat.png");
								stone = loadTexture("resource\\stone.png");
								large_soil = loadTexture("resource\\two_soil.png");
								ground_cone_small = loadTexture("resource\\soil_cone_small.png");
								hill = loadTexture("resource\\hill.png");
								line = loadTexture("resource\\line.png");
								velocity_arrow = loadTexture("resource\\velocity_arrow.png");
								running_end_menu = loadTexture("resource\\running_end_menu.png");
								save_map = loadTexture("resource\\save.png");
								back_to_map = loadTexture("resource\\back_to_map.png");
								line_drag = loadTexture("resource\\line_drag.png");
								map_edit_earthquake = loadTexture("resource\\map_edit_earthquake.png");
								small_arrow = loadTexture("resource\\small_arrow.png");
								LR_img = loadTexture("resource\\LR_img.png");
								UD_img = loadTexture("resource\\UD_img.png");
								UDLR_img = loadTexture("resource\\UDLR_img.png");
								rotation_img = loadTexture("resource\\rotation_img.png");
								circle_img = loadTexture("resource\\circle_img.png");
								swing_img = loadTexture("resource\\swing_img.png");
								reddot = loadTexture("resource\\reddot.png");
								input_bar = loadTexture("resource\\input_bar.png");
								map_editing = true;
								map_load = true;
								map_number = 3;
								Mix_PlayChannel(-1, click_sound, 0);
								Mix_HaltMusic();
								Mix_FadeInMusic(in_stage_music, -1, 1000);
							}
							else if (x >= 109 && x <= 183 && y >= 529 && y <= 626 && map1load == true && map1_delete==false) { // map1 trash click
								map1_delete = true;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 135 && x <= 211 && y >= 550 && y <= 596 && map1load == true && map1_delete == true) { // map 1 delete 125, 550
								map1_delete = false;
								int file_delete;
								file_delete = remove("map1.dat");
								map1load = false;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 295 && x <= 420 && y >= 550 && y <= 596 && map1load == true && map1_delete == true) { // map1 delete cancle
								map1_delete = false;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 528 && x <= 602 && y >= 529 && y <= 626 && map2load == true && map2_delete == false) { // map2 trash click
								map2_delete = true;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 554 && x <= 630 && y >= 550 && y <= 596 && map2load == true && map2_delete == true) { // map 2 delete 125, 550
								map2_delete = false;
								int file_delete;
								file_delete = remove("map2.dat");
								map2load = false;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 714 && x <= 839 && y >= 550 && y <= 596 && map2load == true && map2_delete == true) { // map2 delete cancle
								map2_delete = false;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 947 && x <= 1021 && y >= 529 && y <= 626 && map3load == true && map3_delete == false) { // map3 trash click
								map3_delete = true;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 973 && x <= 1049 && y >= 550 && y <= 596 && map3load == true && map3_delete == true) { // map 3 delete 125, 550
								map3_delete = false;
								int file_delete;
								file_delete = remove("map3.dat");
								map3load = false;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1133 && x <= 1258 && y >= 550 && y <= 596 && map3load == true && map3_delete == true) { // map3 delete cancle
								map3_delete = false;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 334 && x <= 428 && y >= 532 && y <= 626 && map1load == true && map1_delete == false) { // map 1 running 334, 532
								map_running = true;
								map_number = 1;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 753 && x <= 847 && y >= 532 && y <= 626 && map2load == true && map2_delete == false) { // map 2 running
								map_running = true;
								map_number = 2;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1172 && x <= 1266 && y >= 532 && y <= 626 && map3load == true && map3_delete == false) { // map 3 running
								map_running = true;
								map_number = 3;
								Mix_PlayChannel(-1, click_sound, 0);
							}
						}
						break;
					case SDL_MOUSEMOTION:
						int x, y;
						x = event.motion.x;
						y = event.motion.y;
						if (x >= 1135 && x <= 1293 && y >= 57 && y <= 119) { // on next button
							nextbutton = true;
						}
						else if (x >= 975 && x <= 1322 && y >= 669 && y <= 740) { // on mainmenu button
							mainmenubutton = true;
						}
						else if (x >= 109 && x <= 183 && y >= 529 && y <= 626 && map1_delete == false && map1load == true) { // on map1 trash bin 74 97
							map1trash_button = true;
						}
						else if (x >= 216 && x <= 310 && y >= 532 && y <= 626 && map1_delete == false) { // on map1 write 94 94
							map1write_button = true;
						}
						else if (x >= 334 && x <= 428 && y >= 532 && y <= 626 && map1_delete == false && map1load == true) { // on map1 play 334, 532, 94, 94
							map1play_button = true;
						}
						else if (x >= 528 && x <= 602 && y >= 529 && y <= 626 && map2_delete == false && map2load == true) { // on map2 trash bin 74 97
							map2trash_button = true;
						}
						else if (x >= 635 && x <= 729 && y >= 532 && y <= 626 && map2_delete == false) { // on map2 write 94 94
							map2write_button = true;
						}
						else if (x >= 753 && x <= 847 && y >= 532 && y <= 626 && map2_delete == false && map2load == true) { // on map2 play 334, 532, 94, 94
							map2play_button = true;
						}
						else if (x >= 947 && x <= 1021 && y >= 529 && y <= 626 && map3_delete == false && map3load == true) { // on map3 trash bin 74 97
							map3trash_button = true;
						}
						else if (x >= 1054 && x <= 1148 && y >= 532 && y <= 626 && map3_delete == false) { // on map3 write 94 94
							map3write_button = true;
						}
						else if (x >= 1172 && x <= 1266 && y >= 532 && y <= 626 && map3_delete == false && map3load == true) { // on map3 play 334, 532, 94, 94
							map3play_button = true;
						}
						else if (x >= 135 && x <= 211 && y >= 550 && y <= 596 && map1load == true && map1_delete == true) {
							map1yes_button = true;
						}
						else if (x >= 295 && x <= 420 && y >= 550 && y <= 596 && map1load == true && map1_delete == true) {
							map1no_button = true;
						}
						else if (x >= 554 && x <= 630 && y >= 550 && y <= 596 && map2load == true && map2_delete == true) {
							map2yes_button = true;
						}
						else if (x >= 714 && x <= 839 && y >= 550 && y <= 596 && map2load == true && map2_delete == true) {
							map2no_button = true;
						}
						else if (x >= 973 && x <= 1049 && y >= 550 && y <= 596 && map3load == true && map3_delete == true) {
							map3yes_button = true;
						}
						else if (x >= 1133 && x <= 1258 && y >= 550 && y <= 596 && map3load == true && map3_delete == true) {
							map3no_button = true;
						}
						else {
							nextbutton = false;
							mainmenubutton = false;
							map1yes_button = false;
							map2yes_button = false;
							map3yes_button = false;
							map1no_button = false;
							map2no_button = false;
							map3no_button = false;

							map1trash_button = false;
							map1write_button = false;
							map1play_button = false;
							map2trash_button = false;
							map2write_button = false;
							map2play_button = false;
							map3trash_button = false;
							map3write_button = false;
							map3play_button = false;
						}
						break;
					}
				}
				drawTexture(renderer, 0, 0, map_editor_basic);
				drawTexture(renderer, 478, 56, map_editor_menu);
				drawTexture(renderer, 975, 669, mainmenu_button);
				drawTexture(renderer, 1135, 57, next);

				if (nextbutton == true) { // on next button
					stretchTextureEx(renderer, 1070, 55, 65, 65, arrow, 0);
				}
				else if (mainmenubutton == true) { // on mainmenu button
					stretchTextureEx(renderer, 910, 667, 65, 65, arrow, 0);
				}

				if (map1load == true && map1_delete==false) {

					sprintf(text_string, "name");
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 120, 300, text);
					drawTexture(renderer, 120, 300, text);
					drawTexture(renderer, 120, 300, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "- %s", map1_name);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 120, 330, text);
					drawTexture(renderer, 120, 330, text);
					drawTexture(renderer, 120, 330, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "date modified");
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 120, 400, text);
					drawTexture(renderer, 120, 400, text);
					drawTexture(renderer, 120, 400, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "- %d.%02d.%02d", map1_year, map1_month, map1_day);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 120, 430, text);
					drawTexture(renderer, 120, 430, text);
					drawTexture(renderer, 120, 430, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "  %02d : %02d : %02d", map1_hour, map1_min, map1_sec);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 120, 460, text);
					drawTexture(renderer, 120, 460, text);
					drawTexture(renderer, 120, 460, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					if(map1trash_button == false)drawTexture(renderer, 109, 529, trash_bin);
					else stretchTextureEx(renderer, 109, 529, 74, 97, trash_bin, -10);
					if(map1write_button == false)drawTexture(renderer, 216, 532, write);
					else stretchTextureEx(renderer, 216, 532, 94, 94, write, -10);
					if(map1play_button == false) drawTexture(renderer, 334, 532, play);
					else stretchTextureEx(renderer, 334, 532, 94, 94, play, -10);
				}
				else if (map1load == false) {
					if (map1write_button == false) drawTexture(renderer, 217, 539, write);
					else stretchTextureEx(renderer, 217, 539, 94, 94, write, -10);
				}
				else if (map1load == true && map1_delete == true) {

					sprintf(text_string, "Do you really want to");
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 105, 350, text);
					drawTexture(renderer, 105, 350, text);
					drawTexture(renderer, 105, 350, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "delete '%s'?", map1_name);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 105, 390, text);
					drawTexture(renderer, 105, 390, text);
					drawTexture(renderer, 105, 390, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					if (map1yes_button == true) drawTexture(renderer, 75, 540, arrow);
					sprintf(text_string, "Yes!");
					text_surface = TTF_RenderText_Blended(font40, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 135, 550, text);
					drawTexture(renderer, 135, 550, text);
					drawTexture(renderer, 135, 550, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					if (map1no_button == true) drawTexture(renderer, 235, 540, arrow);
					sprintf(text_string, "Noooo");
					text_surface = TTF_RenderText_Blended(font40, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 295, 550, text);
					drawTexture(renderer, 295, 550, text);
					drawTexture(renderer, 295, 550, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
				}

				if (map2load == true && map2_delete == false) {

					sprintf(text_string, "name");
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 539, 300, text);
					drawTexture(renderer, 539, 300, text);
					drawTexture(renderer, 539, 300, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "- %s", map2_name);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 539, 330, text);
					drawTexture(renderer, 539, 330, text);
					drawTexture(renderer, 539, 330, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "date modified");
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 539, 400, text);
					drawTexture(renderer, 539, 400, text);
					drawTexture(renderer, 539, 400, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "- %d.%02d.%02d", map2_year, map2_month, map2_day);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 539, 430, text);
					drawTexture(renderer, 539, 430, text);
					drawTexture(renderer, 539, 430, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "  %02d : %02d : %02d", map2_hour, map2_min, map2_sec);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 539, 460, text);
					drawTexture(renderer, 539, 460, text);
					drawTexture(renderer, 539, 460, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					if (map2trash_button == false)drawTexture(renderer, 528, 529, trash_bin);
					else stretchTextureEx(renderer, 528, 529, 74, 97, trash_bin, -10);
					if (map2write_button == false)drawTexture(renderer, 635, 532, write);
					else stretchTextureEx(renderer, 635, 532, 94, 94, write, -10);
					if (map2play_button == false) drawTexture(renderer, 753, 532, play);
					else stretchTextureEx(renderer, 753, 532, 94, 94, play, -10);
				}
				else if (map2load == false) {
					if (map2write_button == false) drawTexture(renderer, 636, 539, write);
					else stretchTextureEx(renderer, 636, 539, 94, 94, write, -10);
				}
				else if (map2load == true && map2_delete == true) {

					sprintf(text_string, "Do you really want to");
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 524, 350, text);
					drawTexture(renderer, 524, 350, text);
					drawTexture(renderer, 524, 350, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "delete '%s'?", map2_name);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 524, 390, text);
					drawTexture(renderer, 524, 390, text);
					drawTexture(renderer, 524, 390, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					if (map2yes_button == true) drawTexture(renderer, 494, 540, arrow);
					sprintf(text_string, "Yes!");
					text_surface = TTF_RenderText_Blended(font40, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 554, 550, text);
					drawTexture(renderer, 554, 550, text);
					drawTexture(renderer, 554, 550, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					if (map2no_button == true) drawTexture(renderer, 654, 540, arrow);
					sprintf(text_string, "Noooo");
					text_surface = TTF_RenderText_Blended(font40, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 714, 550, text);
					drawTexture(renderer, 714, 550, text);
					drawTexture(renderer, 714, 550, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
				}

				if (map3load == true && map3_delete == false) {

					sprintf(text_string, "name");
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 958, 300, text);
					drawTexture(renderer, 958, 300, text);
					drawTexture(renderer, 958, 300, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "- %s", map3_name);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 958, 330, text);
					drawTexture(renderer, 958, 330, text);
					drawTexture(renderer, 958, 330, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "date modified");
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 958, 400, text);
					drawTexture(renderer, 958, 400, text);
					drawTexture(renderer, 958, 400, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "- %d.%02d.%02d", map3_year, map3_month, map3_day);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 958, 430, text);
					drawTexture(renderer, 958, 430, text);
					drawTexture(renderer, 958, 430, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "  %02d : %02d : %02d", map3_hour, map3_min, map3_sec);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 958, 460, text);
					drawTexture(renderer, 958, 460, text);
					drawTexture(renderer, 958, 460, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					if (map3trash_button == false)drawTexture(renderer, 947, 529, trash_bin);
					else stretchTextureEx(renderer, 947, 529, 74, 97, trash_bin, -10);
					if (map3write_button == false)drawTexture(renderer, 1054, 532, write);
					else stretchTextureEx(renderer, 1054, 532, 94, 94, write, -10);
					if (map3play_button == false) drawTexture(renderer, 1172, 532, play);
					else stretchTextureEx(renderer, 1172, 532, 94, 94, play, -10);
				}
				else if (map3load == false) {
					if (map3write_button == false) drawTexture(renderer, 1055, 539, write);
					else stretchTextureEx(renderer, 1055, 539, 94, 94, write, -10);
				}
				else if (map3load == true && map3_delete == true) {

					sprintf(text_string, "Do you really want to");
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 943, 350, text);
					drawTexture(renderer, 943, 350, text);
					drawTexture(renderer, 943, 350, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					sprintf(text_string, "delete '%s'?", map3_name);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 943, 390, text);
					drawTexture(renderer, 943, 390, text);
					drawTexture(renderer, 943, 390, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					if (map3yes_button == true) drawTexture(renderer, 913, 540, arrow);
					sprintf(text_string, "Yes!");
					text_surface = TTF_RenderText_Blended(font40, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 973, 550, text);
					drawTexture(renderer, 973, 550, text);
					drawTexture(renderer, 973, 550, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					if (map3no_button == true) drawTexture(renderer, 1073, 540, arrow);
					sprintf(text_string, "Noooo");
					text_surface = TTF_RenderText_Blended(font40, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1133, 550, text);
					drawTexture(renderer, 1133, 550, text);
					drawTexture(renderer, 1133, 550, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
				}

				if (next_click == false && escape_map_editor == false) SDL_RenderPresent(renderer);
				total_frame_end = SDL_GetPerformanceCounter();
				total_time = (float)(total_frame_end - total_frame_start) / ((float)SDL_GetPerformanceFrequency());
				total_delay_time = (int)(((1 / fpss)*rendering_factor - total_time) * 1000);

				if (total_delay_time>0) SDL_Delay(total_delay_time);
				else SDL_Delay(1);
				SDL_RenderClear(renderer);
			}

			while (quit == false && scene == map_edit &&map_editing == true) {
				total_frame_start = SDL_GetPerformanceCounter();
				if (map_load == true) {
					FILE * load_file;
					switch (map_number) {
					case 1:
						load_file = fopen("map1.dat", "rb");

						free(map_name);
						fread(&map_name_length, sizeof(map_name_length), 1, load_file);
						map_name = (char *)malloc(sizeof(char)*map_name_length);
						for (i = 0; i < map_name_length; i++) fread(&map_name[i], sizeof(map_name[i]), 1, load_file);
						fread(&map1_year, sizeof(map1_year), 1, load_file);
						fread(&map1_month, sizeof(map1_month), 1, load_file);
						fread(&map1_day, sizeof(map1_day), 1, load_file);
						fread(&map1_hour, sizeof(map1_hour), 1, load_file);
						fread(&map1_min, sizeof(map1_min), 1, load_file);
						fread(&map1_sec, sizeof(map1_sec), 1, load_file);
						break;
					case 2:
						load_file = fopen("map2.dat", "rb");

						free(map_name);
						fread(&map_name_length, sizeof(map_name_length), 1, load_file);
						map_name = (char *)malloc(sizeof(char)*map_name_length);
						for (i = 0; i < map_name_length; i++) fread(&map_name[i], sizeof(map_name[i]), 1, load_file);
						fread(&map2_year, sizeof(map2_year), 1, load_file);
						fread(&map2_month, sizeof(map2_month), 1, load_file);
						fread(&map2_day, sizeof(map2_day), 1, load_file);
						fread(&map2_hour, sizeof(map2_hour), 1, load_file);
						fread(&map2_min, sizeof(map2_min), 1, load_file);
						fread(&map2_sec, sizeof(map2_sec), 1, load_file);
						break;
					case 3:
						load_file = fopen("map3.dat", "rb");

						free(map_name);
						fread(&map_name_length, sizeof(map_name_length), 1, load_file);
						map_name = (char *)malloc(sizeof(char)*map_name_length);
						for (i = 0; i < map_name_length; i++) fread(&map_name[i], sizeof(map_name[i]), 1, load_file);
						fread(&map3_year, sizeof(map3_year), 1, load_file);
						fread(&map3_month, sizeof(map3_month), 1, load_file);
						fread(&map3_day, sizeof(map3_day), 1, load_file);
						fread(&map3_hour, sizeof(map3_hour), 1, load_file);
						fread(&map3_min, sizeof(map3_min), 1, load_file);
						fread(&map3_sec, sizeof(map3_sec), 1, load_file);
						break;
					}
					fread(&population, sizeof(population), 1, load_file);
					object = (Object *)malloc(sizeof(Object)*population);
					for (i = 0; i < population; i++) {
						int shape_number;
						char material_number;
						float x1, x2, v1, v2, a1, a2, th, w, alp, F1, F2;
						fread(&shape_number, sizeof(shape_number), 1, load_file);
						fread(&material_number, sizeof(material_number), 1, load_file);
						fread(&x1, sizeof(x1), 1, load_file);
						fread(&x2, sizeof(x2), 1, load_file);
						fread(&v1, sizeof(v1), 1, load_file);
						fread(&v2, sizeof(v2), 1, load_file);
						fread(&a1, sizeof(a1), 1, load_file);
						fread(&a2, sizeof(a2), 1, load_file);
						fread(&th, sizeof(th), 1, load_file);
						fread(&w, sizeof(w), 1, load_file);
						fread(&alp, sizeof(alp), 1, load_file);
						fread(&F1, sizeof(F1), 1, load_file);
						fread(&F2, sizeof(F2), 1, load_file);
						set_object(&object[i], shape_number, material_number, x1*trans, x2*trans, v1*trans, v2*trans, a1*trans, a2*trans, th, w, alp, F1, F2);
					}
					fread(&budget, sizeof(budget), 1, load_file);
					print_digit(budget, &hundreds, &tenth, &units);
					fread(&height, sizeof(height), 1, load_file);
					fread(&time_duration, sizeof(time_duration), 1, load_file);
					fread(&earthquake_type, sizeof(earthquake_type), 1, load_file);
					fread(&amplitude, sizeof(amplitude), 1, load_file);
					fread(&frequency, sizeof(frequency), 1, load_file);
					fread(&bomb_population, sizeof(bomb_population), 1, load_file);
					bomb = (Bomb *)malloc(sizeof(Bomb)*bomb_population);
					for (i = 0; i < bomb_population; i++) {
						fread(&bomb[i].x[0], sizeof(bomb[i].x[0]), 1, load_file);
						fread(&bomb[i].x[1], sizeof(bomb[i].x[1]), 1, load_file);
						fread(&bomb[i].momentum, sizeof(bomb[i].momentum), 1, load_file);
					}
					map_load = false;
					fclose(load_file);
				}

				while (SDL_PollEvent(&event)) {
					if (event.type == SDL_QUIT) quit = true;
					if (event.type == SDL_MOUSEBUTTONDOWN) {
						if (event.button.button == SDL_BUTTON_LEFT && save_screen ==false) {
							int x, y;
							x = event.motion.x;
							y = event.motion.y;
							if (x >= 1108 && x <= 1343 && y >= 709 && y <= 735) { // click back to map button
								SDL_DestroyTexture(editor_back);
								SDL_DestroyTexture(right_menu);
								SDL_DestroyTexture(back_to_stage);
								SDL_DestroyTexture(pre_wood_one_block);
								SDL_DestroyTexture(pre_wood_two_block);
								SDL_DestroyTexture(pre_wood_three_block);
								SDL_DestroyTexture(pre_wood_hexagon_block);
								SDL_DestroyTexture(pre_wood_polygon1_block);
								SDL_DestroyTexture(pre_wood_trapezoid_block);
								SDL_DestroyTexture(pre_metal_one_block);
								SDL_DestroyTexture(pre_metal_two_block);
								SDL_DestroyTexture(pre_metal_three_block);
								SDL_DestroyTexture(pre_metal_hexagon_block);
								SDL_DestroyTexture(pre_metal_polygon1_block);
								SDL_DestroyTexture(pre_metal_trapezoid_block);
								SDL_DestroyTexture(pre_rock_one_block);
								SDL_DestroyTexture(pre_rock_two_block);
								SDL_DestroyTexture(pre_rock_three_block);
								SDL_DestroyTexture(pre_rock_hexagon_block);
								SDL_DestroyTexture(pre_rock_polygon1_block);
								SDL_DestroyTexture(pre_rock_trapezoid_block);
								SDL_DestroyTexture(bombimg);
								SDL_DestroyTexture(cat);
								SDL_DestroyTexture(stone);
								SDL_DestroyTexture(large_soil);
								SDL_DestroyTexture(ground_cone_small);
								SDL_DestroyTexture(hill);
								SDL_DestroyTexture(line);
								SDL_DestroyTexture(velocity_arrow);
								SDL_DestroyTexture(running_end_menu);
								SDL_DestroyTexture(save_map);
								SDL_DestroyTexture(back_to_map);
								SDL_DestroyTexture(line_drag);
								SDL_DestroyTexture(map_edit_earthquake);
								SDL_DestroyTexture(small_arrow);
								SDL_DestroyTexture(small_arrow);
								SDL_DestroyTexture(LR_img);
								SDL_DestroyTexture(UD_img);
								SDL_DestroyTexture(UDLR_img);
								SDL_DestroyTexture(rotation_img);
								SDL_DestroyTexture(circle_img);
								SDL_DestroyTexture(swing_img);
								SDL_DestroyTexture(reddot);
								SDL_DestroyTexture(input_bar);
								map_editing = false;
								escape_map_editor = true;
								if (population != 0) {
									for (i = 0; i < population; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
										free(object[i].shape->vertex); free(object[i].shape->normal);
										free(object[i].shape->vertex_contact_index);
										free(object[i].shape->vertex_contact);
										free(object[i].shape);
									}
									free(object);
								}
								population = 0;
								if (bomb_population != 0) free(bomb);
								bomb_population = 0;
								free(map_name);
								Mix_PlayChannel(-1, click_sound, 0);
								Mix_HaltMusic();
								Mix_FadeInMusic(main_music, -1, 1000);
							}
							else if (x >= 1124 && x <= 1124 + Length*trans && y >= 100 && y <= 100 + Length*trans && material !=7) { // click one block
								click_one_block = true;
							}
							else if (x >= 1110 && x <= 1180 && y >= 80 && y <= 150 && material == 7) {
								click_one_block = true;
							}
							else if (x >= 1234 && x <= 1234 + 2 * Length*trans && y >= 100 && y <= 100 + Length*trans && material !=7) { // click two block
								click_two_block = true;
							}
							else if(x >= 1224 && x <= 1334 && y >= 110 && y <= 138 && material == 7){
								click_two_block = true;
							}
							else if (x >= 1084 && x <= 1084 + 3 * Length*trans && y >= 216 && y <= 216 + Length*trans && material != 7) { // click three block
								click_three_block = true;
							}
							else if (x >= 1094 && x <= 1195 && y >= 211 && y <= 280 && material == 7) {
								click_three_block = true;
							}
							else if (x >= 1235 && x <= 1235 + 2 * Length*trans && y >= 192 && y <= 192 + Length*trans*sqrt(3) && material != 7) { // click hexagon block
								click_hexagon_block = true;
							}
							else if (x >= 1224 && x <= 1324 && y >= 221 && y <= 258 && material == 7) {
								click_hexagon_block = true;
							}
							else if (x >= 1104 && x <= 1104 + 2 * Length*trans && y >= 308 && y <= 308 + 2 * Length*trans && material != 7) { // click polygon1 block
								click_polygon1_block = true;
							}
							else if (x >= 1114 && x <= 1165 && y >= 327 && y <= 389 && material == 7) {
								click_polygon1_block = true;
							}
							else if (x >= 1235 && x <= 1235 + 2 * Length*trans && y >= 347 && y <= 347 + Length*trans*sqrt(3) / 2.0 && material != 7) {
								click_trapezoid_block = true;
							}
							else if (x >= 1224 && x <= 1324 && y >= 305 && y <= 417 && material == 7) {
								click_trapezoid_block = true;
							}
							else if (x >= 961 && x<977 && y>height - 15 && y < height + 17) {
								click_line = true;
								click_y = mouse_y;
							}
							else if (x >= 1216 && x < 1238 && y >= 440 && y < 468 && (material == 5 || material == 4 || material == 6)) {
								if (amplitude == 5.0f) amplitude = 4.0f;
								else if (amplitude == 4.0f) amplitude = 3.0f;
								else if (amplitude == 3.0f) amplitude = 2.0f;
								else if (amplitude == 2.0f) amplitude = 1.0f;
								else if (amplitude == 1.0f) amplitude = 0.9f;
								else if (amplitude == 0.9f) amplitude = 0.7f;
								else if (amplitude == 0.7f) amplitude = 0.5f;
								else if (amplitude == 0.5f) amplitude = 0.3f;
								else if (amplitude == 0.3f) amplitude = 0.1f;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1320 && x < 1342 && y >= 440 && y < 468 && (material == 5 || material == 4 || material == 6)) {
								if (amplitude == 0.1f) amplitude = 0.3f;
								else if (amplitude == 0.3f) amplitude = 0.5f;
								else if (amplitude == 0.5f) amplitude = 0.7f;
								else if (amplitude == 0.7f) amplitude = 0.9f;
								else if (amplitude == 0.9f) amplitude = 1.0f;
								else if (amplitude == 1.0f) amplitude = 2.0f;
								else if (amplitude == 2.0f) amplitude = 3.0f;
								else if (amplitude == 3.0f) amplitude = 4.0f;
								else if (amplitude = 4.0f) amplitude = 5.0f;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1176 && x < 1198 && y >= 485 && y < 513 && (material == 5 || material == 4 || material == 6)) { // 1176, 485, 22, 28
								if (earthquake_type > 0)earthquake_type = earthquake_type - 1;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1320 && x < 1342 && y >= 485 && y < 513 && (material == 5 || material == 4 || material == 6)) { // 1320, 485, 22, 28
								if (earthquake_type < 8)earthquake_type = earthquake_type + 1;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1203 && x < 1225 && y >= 535 && y < 563 && (material == 5 || material == 4 || material == 6)) {
								if(time_duration>1.5f) time_duration = time_duration - 0.5f;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1320 && x < 1342 && y >= 535 && y < 563 && (material == 5 || material == 4 || material == 6)) {
								if (time_duration<10.5f) time_duration = time_duration + 0.5f;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1216 && x < 1238 && y >= 585 && y < 613 && (material == 5 || material == 4 || material == 6)) {
								if (frequency == 2.0f) frequency = 1.5f;
								else if (frequency == 1.5f)frequency = 1.0f;
								else if (frequency == 1.0f) frequency = 0.7f;
								else if (frequency == 0.7f)frequency = 0.5f;
								else if (frequency == 0.5f)frequency = 0.3f;
								else if (frequency = 0.3f) frequency = 0.1f;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1320 && x < 1342 && y >= 585 && y < 613 && (material == 5 || material == 4 || material == 6)) {
								if (frequency == 0.1f) frequency = 0.3f;
								else if (frequency == 0.3f)frequency = 0.5f;
								else if (frequency == 0.5f) frequency = 0.7f;
								else if (frequency == 0.7f)frequency = 1.0f;
								else if (frequency == 1.0f)frequency = 1.5f;
								else if (frequency = 1.5f) frequency = 2.0f;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1176 && x < 1204 && y >= 432 && y < 454 && material == 7 && budget<999) {
								budget = budget + 100;
								if (budget > 999) budget = 999;
								print_digit(budget, &hundreds, &tenth, &units);
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1176 && x < 1204 && y >= 487 && y < 509 && material == 7 && budget>0) {
								budget = budget - 100;
								if (budget < 0) budget = 0;
								print_digit(budget, &hundreds, &tenth, &units);
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1216 && x < 1244 && y >= 432 && y < 454 && material == 7 && budget<999) {
								budget = budget + 10;
								if (budget > 999) budget = 999;
								print_digit(budget, &hundreds, &tenth, &units);
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1216 && x < 1244 && y >= 487 && y < 509 && material == 7 && budget>0) {
								budget = budget - 10;
								if (budget < 0) budget = 0;
								print_digit(budget, &hundreds, &tenth, &units);
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1256 && x < 1284 && y >= 432 && y < 454 && material == 7 && budget<999) {
								budget = budget + 1;
								if (budget > 999) budget = 999;
								print_digit(budget, &hundreds, &tenth, &units);
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1256 && x < 1284 && y >= 487 && y < 509 && material == 7 && budget>0) {
								budget = budget - 1;
								if (budget < 0) budget = 0;
								print_digit(budget, &hundreds, &tenth, &units);
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x >= 1125 && x < 1288 && y >= 632 && y < 679) { // click_save
								/*
								timer_ = time(NULL);
								tp = localtime(&timer_);
								save_year= tp->tm_year + 1900;
								save_month= tp->tm_mon + 1;
								save_day= tp->tm_mday;
								save_hour= tp->tm_hour;
								save_min= tp->tm_min;
								save_sec = tp->tm_sec;
								*/
								save_screen = true;
								map_editor_save_button = false;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else {
								for (i = 0; i < population; i++) if (click_perception(mouse_x / trans, mouse_y / trans, &object[i]) < 0) {
									if (copy == true && (object[i].shape->number != 95 && object[i].shape->number != 97 && object[i].shape->number != 96)) {
										click_block = i; click_x = mouse_x; click_y = mouse_y;

										tempt_object = (Object *)malloc(sizeof(Object)*population);
										for (i = 0; i < population; i++) set_object(&tempt_object[i], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
										if (population != 0) {
											for (i = 0; i < population; i++) {
												for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
												free(object[i].shape->vertex); free(object[i].shape->normal);
												free(object[i].shape->vertex_contact_index);
												free(object[i].shape->vertex_contact);
												free(object[i].shape);
											}
											free(object);
										}
										population = population + 1;
										object = (Object *)malloc(sizeof(Object)*population);
										for (i = 0; i < population - 1; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
										set_object(&object[population - 1], object[click_block].shape->number, object[click_block].material.number, object[click_block].x[0] * trans, object[click_block].x[1] * trans, object[click_block].v[0] * trans, object[click_block].v[1] * trans, 0, 0, object[click_block].th, 0, 0, 0, 0);
										if (population - 1 != 0) {
											for (i = 0; i < population - 1; i++) {
												for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
												free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
												free(tempt_object[i].shape->vertex_contact_index);
												free(tempt_object[i].shape->vertex_contact);
												free(tempt_object[i].shape);
											}
											free(tempt_object);
										}
										for (i = 0; i < population; i++) reassign_vertex(&object[i]);
									}
									else if (copy == true && (object[i].shape->number == 95 || object[i].shape->number == 97 || object[i].shape->number == 96)) {
										if(object[i].shape->number == 97) click_block = i;
										else if (object[i].shape->number == 95) click_block = i + 1;
										else if (object[i].shape->number == 96) click_block = i - 1;
										click_x = mouse_x; click_y = mouse_y;

										tempt_object = (Object *)malloc(sizeof(Object)*population);
										for (i = 0; i < population; i++) set_object(&tempt_object[i], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
										if (population != 0) {
											for (i = 0; i < population; i++) {
												for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
												free(object[i].shape->vertex); free(object[i].shape->normal);
												free(object[i].shape->vertex_contact_index);
												free(object[i].shape->vertex_contact);
												free(object[i].shape);
											}
											free(object);
										}
										population = population + 3;
										object = (Object *)malloc(sizeof(Object)*population);
										for (i = 0; i < population - 3; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
										set_object(&object[population - 3], object[click_block - 1].shape->number, object[click_block - 1].material.number, object[click_block - 1].x[0] * trans, object[click_block - 1].x[1] * trans, 0, 0, 0, 0, object[click_block - 1].th, 0, 0, 0, 0);
										set_object(&object[population - 2], object[click_block].shape->number, object[click_block].material.number, object[click_block].x[0] * trans, object[click_block].x[1] * trans, 0, 0, 0, 0, object[click_block].th, 0, 0, 0, 0);
										set_object(&object[population - 1], object[click_block + 1].shape->number, object[click_block + 1].material.number, object[click_block + 1].x[0] * trans, object[click_block + 1].x[1] * trans, 0, 0, 0, 0, object[click_block + 1].th, 0, 0, 0, 0);
										if (population - 3 != 0) {
											for (i = 0; i < population - 3; i++) {
												for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
												free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
												free(tempt_object[i].shape->vertex_contact_index);
												free(tempt_object[i].shape->vertex_contact);
												free(tempt_object[i].shape);
											}
											free(tempt_object);
										}
										for (i = 0; i < population; i++) reassign_vertex(&object[i]);
									}
									else {
										click_block = i; 
										click_x = mouse_x; 
										click_y = mouse_y;
										if (object[click_block].shape->number == 95) click_block = click_block + 1;
										else if (object[click_block].shape->number == 96) click_block = click_block - 1;
									}
								}
								if (click_block == -1) {
									for (i = 0; i < bomb_population; i++) if ((bomb[i].x[0] * trans - mouse_x)*(bomb[i].x[0] * trans - mouse_x) + (bomb[i].x[1] * trans - mouse_y)*(bomb[i].x[1] * trans - mouse_y) < 625) {
										if (copy == true) {
											click_bomb = i; click_x = mouse_x; click_y = mouse_y;

											tempt_bomb = (Bomb *)malloc(sizeof(Bomb)*bomb_population);
											for (i = 0; i < bomb_population; i++) set_bomb(&tempt_bomb[i], bomb[i].x[0] * trans, bomb[i].x[1] * trans, 50.0);
											if (bomb_population != 0) free(bomb);

											bomb_population = bomb_population + 1;
											bomb = (Bomb *)malloc(sizeof(Bomb)*bomb_population);
											for (i = 0; i < bomb_population - 1; i++) set_bomb(&bomb[i], tempt_bomb[i].x[0] * trans, tempt_bomb[i].x[1] * trans, 50.0);
											set_bomb(&bomb[bomb_population - 1], bomb[click_bomb].x[0] * trans, bomb[click_bomb].x[1] * trans, 50.0);
											if (population - 1 != 0) free(tempt_bomb);
										}
										else click_bomb = i; click_x = mouse_x; click_y = mouse_y;
									}
								}
							}
						}
						else if (event.button.button == SDL_BUTTON_LEFT && save_screen == true) {
							int x, y;
							x = event.motion.x;
							y = event.motion.y;
							if (x > 590 && x <= 813 && y > 450 && y <= 531 && save_screen == true) {
								save_screen = false;
								Mix_PlayChannel(-1, click_sound, 0);
							}
							else if (x > 310 && x <= 462 && y > 450 && y <= 531 && save_screen == true) {
								FILE * file_pointer;

								timer_ = time(NULL);
								tp = localtime(&timer_);
								save_year = tp->tm_year + 1900;
								save_month = tp->tm_mon + 1;
								save_day = tp->tm_mday;
								save_hour = tp->tm_hour;
								save_min = tp->tm_min;
								save_sec = tp->tm_sec;

								switch (map_number) {
								case(1):
									map1 = fopen("map1.dat", "wb");
									file_pointer = map1;
									break;
								case(2):
									map2 = fopen("map2.dat", "wb");
									file_pointer = map2;
									break;
								case(3):
									map3 = fopen("map3.dat", "wb");
									file_pointer = map3;
									break;
								}
								fwrite(&map_name_length, sizeof(map_name_length), 1, file_pointer); //fwrite(&b, sizeof(b), 1, fp);
								for (i = 0; i < map_name_length;i++) fwrite(&map_name[i], sizeof(map_name[i]), 1, file_pointer);
								fwrite(&save_year, sizeof(save_year), 1, file_pointer);
								fwrite(&save_month, sizeof(save_month), 1, file_pointer);
								fwrite(&save_day, sizeof(save_day), 1, file_pointer);
								fwrite(&save_hour, sizeof(save_hour), 1, file_pointer);
								fwrite(&save_min, sizeof(save_min), 1, file_pointer);
								fwrite(&save_sec, sizeof(save_sec), 1, file_pointer);

								fwrite(&population, sizeof(population), 1, file_pointer);
								for (i = 0; i < population; i++) {
									fwrite(&object[i].shape->number, sizeof(object[i].shape->number), 1, file_pointer);
									fwrite(&object[i].material.number, sizeof(object[i].material.number), 1, file_pointer);
									fwrite(&object[i].x[0], sizeof(object[i].x[0]), 1, file_pointer);
									fwrite(&object[i].x[1], sizeof(object[i].x[1]), 1, file_pointer);
									fwrite(&object[i].v[0], sizeof(object[i].v[0]), 1, file_pointer);
									fwrite(&object[i].v[1], sizeof(object[i].v[1]), 1, file_pointer);
									fwrite(&object[i].a[0], sizeof(object[i].a[0]), 1, file_pointer);
									fwrite(&object[i].a[1], sizeof(object[i].a[1]), 1, file_pointer);
									fwrite(&object[i].th, sizeof(object[i].th), 1, file_pointer);
									fwrite(&object[i].w, sizeof(object[i].w), 1, file_pointer);
									fwrite(&object[i].alp, sizeof(object[i].alp), 1, file_pointer);
									fwrite(&object[i].F[0], sizeof(object[i].F[0]), 1, file_pointer);
									fwrite(&object[i].F[1], sizeof(object[i].F[1]), 1, file_pointer);
								}
								fwrite(&budget, sizeof(budget), 1, file_pointer);
								fwrite(&height, sizeof(height), 1, file_pointer);
								fwrite(&time_duration, sizeof(time_duration), 1, file_pointer);
								fwrite(&earthquake_type, sizeof(earthquake_type), 1, file_pointer);
								fwrite(&amplitude, sizeof(amplitude), 1, file_pointer);
								fwrite(&frequency, sizeof(frequency), 1, file_pointer);
								fwrite(&bomb_population, sizeof(bomb_population), 1, file_pointer);
								for (i = 0; i < bomb_population; i++) {
									fwrite(&bomb[i].x[0], sizeof(bomb[i].x[0]), 1, file_pointer);
									fwrite(&bomb[i].x[1], sizeof(bomb[i].x[1]), 1, file_pointer);
									fwrite(&bomb[i].momentum, sizeof(bomb[i].momentum), 1, file_pointer);
								}

								fclose(file_pointer);
								save_screen = false;
								Mix_PlayChannel(-1, click_sound, 0);
							}
						}
						else if (event.button.button == SDL_BUTTON_RIGHT) {
							for (k = 0; k < population; k++) {
								if (click_perception(mouse_x / trans, mouse_y / trans, &object[k]) < 0 && click_block == -1) {
									if (object[k].shape->number != 95 && object[k].shape->number != 97 && object[k].shape->number != 96) {
										int i, j;

										population = population - 1;
										tempt_object = (Object *)malloc(sizeof(Object)*population);
										i = 0; j = 0;
										for (i = 0; i < population + 1; i++) if (i != k) {
											set_object(&tempt_object[j], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
											j = j + 1;
										}
										for (i = 0; i < population + 1; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
											free(object[i].shape->vertex); free(object[i].shape->normal);
											free(object[i].shape->vertex_contact_index);
											free(object[i].shape->vertex_contact);
											free(object[i].shape);
										}
										free(object);
										object = (Object *)malloc(sizeof(Object)*population);
										for (i = 0; i < population; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
										for (i = 0; i < population; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
											free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
											free(tempt_object[i].shape->vertex_contact_index);
											free(tempt_object[i].shape->vertex_contact);
											free(tempt_object[i].shape);
										}
										free(tempt_object);
										for (i = 0; i < population; i++) reassign_vertex(&object[i]);
										Mix_PlayChannel(-1, erase, 0);
										break;
									}
									else if (object[k].shape->number == 95 || object[k].shape->number == 97 || object[k].shape->number == 96) {
										int i, j;
										int click_block_R;
										if (object[k].shape->number == 97) click_block_R = k;
										else if (object[k].shape->number == 95) click_block_R = k + 1;
										else if (object[k].shape->number == 96) click_block_R = k - 1;

										population = population - 3;
										tempt_object = (Object *)malloc(sizeof(Object)*population);
										i = 0; j = 0;
										for (i = 0; i < population + 3; i++) if (i != click_block_R && i != click_block_R -1 && i!= click_block_R +1) {
											set_object(&tempt_object[j], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
											j = j + 1;
										}
										for (i = 0; i < population + 3; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
											free(object[i].shape->vertex); free(object[i].shape->normal);
											free(object[i].shape->vertex_contact_index);
											free(object[i].shape->vertex_contact);
											free(object[i].shape);
										}
										free(object);
										object = (Object *)malloc(sizeof(Object)*population);
										for (i = 0; i < population; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
										for (i = 0; i < population; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
											free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
											free(tempt_object[i].shape->vertex_contact_index);
											free(tempt_object[i].shape->vertex_contact);
											free(tempt_object[i].shape);
										}
										free(tempt_object);
										for (i = 0; i < population; i++) reassign_vertex(&object[i]);
										Mix_PlayChannel(-1, erase, 0);
										break;
									}
								}
							}
							for (k = 0; k < bomb_population; k++) if ((bomb[k].x[0] * trans - mouse_x)*(bomb[k].x[0] * trans - mouse_x) + (bomb[k].x[1] * trans - mouse_y)*(bomb[k].x[1] * trans - mouse_y) < 625 && click_bomb == -1) {
								int i, j;
								bomb_population = bomb_population - 1;
								tempt_bomb = (Bomb *)malloc(sizeof(Bomb)*bomb_population);
								i = 0; j = 0;
								for (i = 0; i < bomb_population + 1; i++) if (i != k) {
									set_bomb(&tempt_bomb[j], bomb[i].x[0] * trans, bomb[i].x[1] * trans, 50.0);
									j = j + 1;
								}
								free(bomb);
								bomb= (Bomb *)malloc(sizeof(Bomb)*bomb_population);
								for(i=0;i<bomb_population;i++) set_bomb(&bomb[i], tempt_bomb[i].x[0] * trans, tempt_bomb[i].x[1] * trans, 50.0);
								free(tempt_bomb);
								Mix_PlayChannel(-1, erase, 0);
								break;
							}
						}
					}
					if (event.type == SDL_MOUSEMOTION) {
						int x, y;
						x = event.motion.x;
						y = event.motion.y;
						mouse_x = x;
						mouse_y = y;

						if (x >= 1108 && x <= 1343 && y >= 709 && y <= 735 && save_screen==false) { // on back to map button
							back_to_map_button = true;
						}
						else if (x > 590 && x <= 813 && y > 450 && y <= 531 && save_screen==true) {
							cancle_mouse = true;
						}
						else if (x > 310 && x <= 462 && y > 450 && y <= 531 && save_screen == true) { // 300, 450
							save_mouse = true;
						}
						else if (x >= 1125 && x < 1288 && y >= 632 && y < 679 && save_screen==false) {
							map_editor_save_button = true;
						}
						else {
							back_to_map_button = false;
							cancle_mouse = false;
							save_mouse = false;
							map_editor_save_button = false;
						}
						if (click_block != -1) {
							object[click_block].x[0] = object[click_block].x[0] + (mouse_x - click_x) / trans;
							object[click_block].x[1] = object[click_block].x[1] + (mouse_y - click_y) / trans;
							if (object[click_block].shape->number == 97) {
								object[click_block + 1].x[0] = object[click_block + 1].x[0] + (mouse_x - click_x) / trans;
								object[click_block + 1].x[1] = object[click_block + 1].x[1] + (mouse_y - click_y) / trans;
								object[click_block - 1].x[0] = object[click_block - 1].x[0] + (mouse_x - click_x) / trans;
								object[click_block - 1].x[1] = object[click_block - 1].x[1] + (mouse_y - click_y) / trans;
							}
							click_x = mouse_x; click_y = mouse_y;
						}
						else if (click_bomb != -1) {
							bomb[click_bomb].x[0] = bomb[click_bomb].x[0] + (mouse_x - click_x) / trans;
							bomb[click_bomb].x[1] = bomb[click_bomb].x[1] + (mouse_y - click_y) / trans;
							click_x = mouse_x; click_y = mouse_y;
						}
						else if (click_line == true) {
							height = height+(mouse_y-click_y);
							click_y = mouse_y;
						}
					}
					if (event.type == SDL_MOUSEBUTTONUP) {
						if ((click_one_block == true && material !=7) || click_two_block == true || click_three_block == true || click_hexagon_block == true || click_polygon1_block == true || (click_trapezoid_block == true && material != 7)) {
							if (mouse_x < 1004) {
								int i, j;
								tempt_object = (Object *)malloc(sizeof(Object)*population);
								for (i = 0; i < population; i++) set_object(&tempt_object[i], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
								if (population != 0) {
									for (i = 0; i < population; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
										free(object[i].shape->vertex); free(object[i].shape->normal);
										free(object[i].shape->vertex_contact_index);
										free(object[i].shape->vertex_contact);
										free(object[i].shape);
									}
									free(object);
								}
								population = population + 1;
								object = (Object *)malloc(sizeof(Object)*population);
								for (i = 0; i < population - 1; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
								if (click_one_block == true) {
									click_one_block = false;
									set_object(&object[population - 1], 1, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
								}
								else if (click_two_block == true) {
									click_two_block = false;
									if (material != 7)set_object(&object[population - 1], 2, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
									else if(material == 7)set_object(&object[population - 1], 99, 99, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
								}
								else if (click_three_block == true) {
									click_three_block = false;
									if (material != 7)set_object(&object[population - 1], 3, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
									else if (material == 7)set_object(&object[population - 1], 80, 98, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
								}
								else if (click_hexagon_block == true) {
									click_hexagon_block = false;
									if(material != 7)set_object(&object[population - 1], 8, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
									else if(material == 7)set_object(&object[population - 1], 98, 99, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
								}
								else if (click_polygon1_block == true) {
									click_polygon1_block = false;
									if(material != 7)set_object(&object[population - 1], 9, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
									else if(material == 7)set_object(&object[population - 1], 81, 97, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
								}
								else if (click_trapezoid_block == true) {
									click_trapezoid_block = false;
									set_object(&object[population - 1], 6, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
								}
								if (population - 1 != 0) {
									for (i = 0; i < population - 1; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
										free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
										free(tempt_object[i].shape->vertex_contact_index);
										free(tempt_object[i].shape->vertex_contact);
										free(tempt_object[i].shape);
									}
									free(tempt_object);
								}
								for (i = 0; i < population; i++) reassign_vertex(&object[i]);
								angle = 0;
								if (material == 5) Mix_PlayChannel(-1, hammering_wood, 0);
								else if (material == 6) Mix_PlayChannel(-1, hammering_iron, 0);
								else if (material == 4)Mix_PlayChannel(-1, hammering_brick, 0);
								else if (material == 7)Mix_PlayChannel(-1, hammering_brick, 0);
							}
							else if (mouse_x >= 1004) {
								click_one_block = false;
								click_two_block = false;
								click_three_block = false;
								click_hexagon_block = false;
								click_polygon1_block = false;
								click_trapezoid_block = false;
								angle = 0;
								Mix_PlayChannel(-1, erase, 0);
							}
						}
						else if (click_one_block == true && material == 7) {
							if (mouse_x < 1004) {
								int i, j;
								tempt_bomb = (Bomb *)malloc(sizeof(Bomb)*bomb_population);
								for (i = 0; i < bomb_population; i++) set_bomb(&tempt_bomb[i], bomb[i].x[0] * trans,bomb[i].x[1] * trans,50.0);
								if (bomb_population != 0) free(bomb);
								bomb_population = bomb_population + 1;
								bomb = (Bomb *)malloc(sizeof(Bomb)*bomb_population);
								for (i = 0; i < bomb_population - 1; i++) set_bomb(&bomb[i], tempt_bomb[i].x[0] * trans, tempt_bomb[i].x[1] * trans, 50.0);
								set_bomb(&bomb[bomb_population - 1], mouse_x, mouse_y, 50.0);
								if (bomb_population != 0) free(tempt_bomb);
								click_one_block = false;
								angle = 0;
								Mix_PlayChannel(-1, hammering_brick, 0);
							} 
							else if (mouse_x >= 1004) {
								click_one_block = false;
								click_two_block = false;
								click_three_block = false;
								click_hexagon_block = false;
								click_polygon1_block = false;
								click_trapezoid_block = false;
								angle = 0;
								Mix_PlayChannel(-1, erase, 0);
							}
						}
						else if (click_trapezoid_block == true && material == 7) {
							if (mouse_x < 1004) {
								int i, j;
								tempt_object = (Object *)malloc(sizeof(Object)*population);
								for (i = 0; i < population; i++) set_object(&tempt_object[i], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
								if (population != 0) {
									for (i = 0; i < population; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
										free(object[i].shape->vertex); free(object[i].shape->normal);
										free(object[i].shape->vertex_contact_index);
										free(object[i].shape->vertex_contact);
										free(object[i].shape);
									}
									free(object);
								}
								population = population + 3;
								object = (Object *)malloc(sizeof(Object)*population);
								for (i = 0; i < population - 3; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
								
								float rotmat[2][2];
								float radius = angle*pi / 180;
								float coordinate[2];
								float output[2];
								set_rot_mat(rotmat, &radius);

								set_object(&object[population - 2], 97, 99, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);

								coordinate[0] = -1.912251*Length*trans; coordinate[1] = -0.102928*Length*trans;
								mat_v_product(rotmat, coordinate, output);
								set_object(&object[population - 3], 95, 99, mouse_x + output[0], mouse_y + output[1], 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);

								coordinate[0] = 2.155681*Length*trans; coordinate[1] = 0.012874*Length*trans;
								mat_v_product(rotmat, coordinate, output);
								set_object(&object[population - 1], 96, 99, mouse_x + output[0], mouse_y + output[1], 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
								/*
								<-1.912251*Length,-0.102928*Length> left<-------middle------->right <2.155681*Length, 0.012874*Lenght>
								*/

								if (population - 3 != 0) {
									for (i = 0; i < population - 3; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
										free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
										free(tempt_object[i].shape->vertex_contact_index);
										free(tempt_object[i].shape->vertex_contact);
										free(tempt_object[i].shape);
									}
									free(tempt_object);
								}
								for (i = 0; i < population; i++) reassign_vertex(&object[i]);
								click_trapezoid_block = false;
								angle = 0;
								Mix_PlayChannel(-1, hammering_brick, 0);
							}
							else if (mouse_x >= 1004) {
								click_one_block = false;
								click_two_block = false;
								click_three_block = false;
								click_hexagon_block = false;
								click_polygon1_block = false;
								click_trapezoid_block = false;
								angle = 0;
								Mix_PlayChannel(-1, erase, 0);
							}
						}
						else if (click_line == true) click_line = false;
						if (click_block != -1) {
							if (object[click_block].x[0] >= 1004 / trans && (object[click_block].shape->number != 97)) {
								int i, j;

								population = population - 1;
								tempt_object = (Object *)malloc(sizeof(Object)*population);
								i = 0; j = 0;
								for (i = 0; i < population + 1; i++) if (i != click_block) {
									set_object(&tempt_object[j], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
									j = j + 1;
								}
								for (i = 0; i < population + 1; i++) {
									for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
									free(object[i].shape->vertex); free(object[i].shape->normal);
									free(object[i].shape->vertex_contact_index);
									free(object[i].shape->vertex_contact);
									free(object[i].shape);
								}
								free(object);
								object = (Object *)malloc(sizeof(Object)*population);
								for (i = 0; i < population; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
								for (i = 0; i < population; i++) {
									for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
									free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
									free(tempt_object[i].shape->vertex_contact_index);
									free(tempt_object[i].shape->vertex_contact);
									free(tempt_object[i].shape);
								}
								free(tempt_object);
								Mix_PlayChannel(-1, erase, 0);
							}
							else if (object[click_block].x[0] >= 1004 / trans && (object[click_block].shape->number == 97)) {
								int i, j;

								population = population - 3;
								tempt_object = (Object *)malloc(sizeof(Object)*population);
								i = 0; j = 0;
								for (i = 0; i < population + 3; i++) if (i != click_block && i != click_block-1 && i != click_block+1) {
									set_object(&tempt_object[j], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
									j = j + 1;
								}
								for (i = 0; i < population + 3; i++) {
									for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
									free(object[i].shape->vertex); free(object[i].shape->normal);
									free(object[i].shape->vertex_contact_index);
									free(object[i].shape->vertex_contact);
									free(object[i].shape);
								}
								free(object);
								object = (Object *)malloc(sizeof(Object)*population);
								for (i = 0; i < population; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);						
								for (i = 0; i < population; i++) {
									for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
									free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
									free(tempt_object[i].shape->vertex_contact_index);
									free(tempt_object[i].shape->vertex_contact);
									free(tempt_object[i].shape);
								}
								free(tempt_object);
								Mix_PlayChannel(-1, erase, 0);
							}
							else {
								if (object[click_block].material.number == 5) Mix_PlayChannel(-1, hammering_wood, 0);
								else if (object[click_block].material.number == 6) Mix_PlayChannel(-1, hammering_iron, 0);
								else if (object[click_block].material.number == 4)Mix_PlayChannel(-1, hammering_brick, 0);
								else Mix_PlayChannel(-1, hammering_brick, 0);
							}
							click_block = -1;
							for (i = 0; i < population; i++) reassign_vertex(&object[i]);
						}
						if (click_bomb != -1) {
							if (bomb[click_bomb].x[0] >= 1004 / trans) {
								int i, j;

								bomb_population = bomb_population - 1;
								tempt_bomb = (Bomb *)malloc(sizeof(Bomb)*bomb_population);
								i = 0; j = 0;
								for (i = 0; i < bomb_population + 1; i++) if (i != click_bomb) {
									set_bomb(&tempt_bomb[j], bomb[i].x[0] * trans, bomb[i].x[1] * trans, 50.0);
									j = j + 1;
								}
								free(bomb);
								bomb = (Bomb *)malloc(sizeof(Bomb)*bomb_population);
								for (i = 0; i < bomb_population; i++) set_bomb(&bomb[i], tempt_bomb[i].x[0]*trans, tempt_bomb[i].x[1] * trans, 50.0);
								free(tempt_bomb);
							}
							click_bomb = -1;
							Mix_PlayChannel(-1, hammering_brick, 0);
						}
					}
					if (event.type == SDL_KEYDOWN) {
						if (save_screen == false) {
							if (event.key.keysym.sym == SDLK_r) {
								if (click_one_block == true || click_two_block == true || click_three_block == true || click_hexagon_block == true || click_polygon1_block == true || click_trapezoid_block == true) { angle = angle - 15; Mix_PlayChannel(-1, gear, 0); }
								if (click_block != -1) { object[click_block].th = object[click_block].th - 15 * pi / 180; Mix_PlayChannel(-1, gear, 0); }
							}
							else if (event.key.keysym.sym == SDLK_t) {
								if (click_one_block == true || click_two_block == true || click_three_block == true || click_hexagon_block == true || click_polygon1_block == true || click_trapezoid_block == true) { angle = angle + 15; Mix_PlayChannel(-1, gear, 0); }
								if (click_block != -1) { object[click_block].th = object[click_block].th + 15 * pi / 180; Mix_PlayChannel(-1, gear, 0); }
							}
							else if (event.key.keysym.sym == SDLK_q) {
								switch (material) {
								case 5: material = 6; break;
								case 6: material = 4; break;
								case 4: material = 7; break;
								case 7: material = 5; break;
								}
							}
							else if (event.key.keysym.sym == SDLK_LCTRL) {
								copy = true;
							}
							else if (event.key.keysym.sym == SDLK_v) {
								int i;
								for (i = 0; i < population; i++) {
									if (object[i].shape->number == 80 && click_perception(mouse_x / trans, mouse_y / trans, &object[i]) < 0) {
										if (object[i].v[0] == 0 && object[i].v[1] == 0) { object[i].v[0] = 11.2f; object[i].v[1] = 0; }
										else if (object[i].v[0] == 11.2f && object[i].v[1] == 0) { object[i].v[0] = 9.7f; object[i].v[1] = -5.6f; }
										else if (object[i].v[0] == 9.7f && object[i].v[1] == -5.6f) { object[i].v[0] = 5.6f; object[i].v[1] = -9.7f; }
										else if (object[i].v[0] == 5.6f && object[i].v[1] == -9.7f) { object[i].v[0] = 0; object[i].v[1] = -11.2f; }
										else if (object[i].v[0] == 0 && object[i].v[1] == -11.2f) { object[i].v[0] = -5.6f; object[i].v[1] = -9.7f; }
										else if (object[i].v[0] == -5.6f && object[i].v[1] == -9.7f) { object[i].v[0] = -9.7f; object[i].v[1] = -5.6f; }
										else if (object[i].v[0] == -9.7f && object[i].v[1] == -5.6f) { object[i].v[0] = -11.2f; object[i].v[1] = 0; }
										else if (object[i].v[0] == -11.2f && object[i].v[1] == 0) { object[i].v[0] = -9.7f; object[i].v[1] = 5.6f; }
										else if (object[i].v[0] == -9.7f && object[i].v[1] == 5.6f) { object[i].v[0] = -5.6f; object[i].v[1] = 9.7f; }
										else if (object[i].v[0] == -5.6f && object[i].v[1] == 9.7f) { object[i].v[0] = 0; object[i].v[1] = 11.2f; }
										else if (object[i].v[0] == 0 && object[i].v[1] == 11.2f) { object[i].v[0] = 5.6f; object[i].v[1] = 9.7f; }
										else if (object[i].v[0] == 5.6f && object[i].v[1] == 9.7f) { object[i].v[0] = 9.7f; object[i].v[1] = 5.6f; }
										else if (object[i].v[0] == 9.7f && object[i].v[1] == 5.6f) { object[i].v[0] = 0; object[i].v[1] = 0; }
										Mix_PlayChannel(-1, gear, 0);
										break;
									}
								}
							}
							else if (event.key.keysym.sym == SDLK_b) {
								int i;
								for (i = 0; i < population; i++) {
									if (object[i].shape->number == 80 && click_perception(mouse_x / trans, mouse_y / trans, &object[i]) < 0) {
										if (object[i].v[0] == 0 && object[i].v[1] == 0) { object[i].v[0] = 9.7f; object[i].v[1] = 5.6f; }
										else if (object[i].v[0] == 9.7f && object[i].v[1] == 5.6f) { object[i].v[0] = 5.6f; object[i].v[1] = 9.7f; }
										else if (object[i].v[0] == 5.6f && object[i].v[1] == 9.7f) { object[i].v[0] = 0; object[i].v[1] = 11.2f; }
										else if (object[i].v[0] == 0 && object[i].v[1] == 11.2f) { object[i].v[0] = -5.6f; object[i].v[1] = 9.7f; }
										else if (object[i].v[0] == -5.6f && object[i].v[1] == 9.7f) { object[i].v[0] = -9.7f; object[i].v[1] = 5.6f; }
										else if (object[i].v[0] == -9.7f && object[i].v[1] == 5.6f) { object[i].v[0] = -11.2f; object[i].v[1] = 0; }
										else if (object[i].v[0] == -11.2f && object[i].v[1] == 0) { object[i].v[0] = -9.7f; object[i].v[1] = -5.6f; }
										else if (object[i].v[0] == -9.7f && object[i].v[1] == -5.6f) { object[i].v[0] = -5.6f; object[i].v[1] = -9.7f; }
										else if (object[i].v[0] == -5.6f && object[i].v[1] == -9.7f) { object[i].v[0] = 0; object[i].v[1] = -11.2f; }
										else if (object[i].v[0] == 0 && object[i].v[1] == -11.2f) { object[i].v[0] = 5.6f; object[i].v[1] = -9.7f; }
										else if (object[i].v[0] == 5.6f && object[i].v[1] == -9.7f) { object[i].v[0] = 9.7f; object[i].v[1] = -5.6f; }
										else if (object[i].v[0] == 9.7f && object[i].v[1] == -5.6f) { object[i].v[0] = 11.2f; object[i].v[1] = 0; }
										else if (object[i].v[0] == 11.2f && object[i].v[1] == 0) { object[i].v[0] = 0; object[i].v[1] = 0; }
										Mix_PlayChannel(-1, gear, 0);
										break;
									}
								}
							}
						}
						else if (save_screen == true) {
							if (((event.key.keysym.sym >= SDLK_a && event.key.keysym.sym <= SDLK_z) || (event.key.keysym.sym >= SDLK_SPACE && event.key.keysym.sym <= SDLK_AT)) && map_name_length<15) {
								char * tempt_name;
								tempt_name = (char *)malloc(sizeof(char)*map_name_length);
								for (i = 0; i < map_name_length; i++) tempt_name[i] = map_name[i];
								free(map_name);
								map_name_length = map_name_length + 1;
								map_name= (char *)malloc(sizeof(char)*map_name_length);
								for (i = 0; i < map_name_length-2; i++) map_name[i] = tempt_name[i];
								map_name[map_name_length - 2] = event.key.keysym.sym;
								map_name[map_name_length - 1] = '\0';
								free(tempt_name);
							}
							else if (event.key.keysym.sym == SDLK_BACKSPACE && map_name_length>1) {
								char * tempt_name;
								tempt_name = (char *)malloc(sizeof(char)*map_name_length);
								for (i = 0; i < map_name_length; i++) tempt_name[i] = map_name[i];
								free(map_name);
								map_name_length = map_name_length - 1;
								map_name = (char *)malloc(sizeof(char)*map_name_length);
								for (i = 0; i < map_name_length - 1; i++) map_name[i] = tempt_name[i];
								map_name[map_name_length - 1] = '\0';
								free(tempt_name);
							}
						}
					}
					if (event.type == SDL_KEYUP) {
						if (event.key.keysym.sym == SDLK_LCTRL) {
							copy = false;
						}
					}
				}

				drawTexture(renderer, 0, 0, editor_back);
				drawTexture(renderer, 0, height, line);
				drawTexture(renderer, 961, height - 15, line_drag);
				drawTexture(renderer, 1004, 0, right_menu);
				drawTexture(renderer, 1125, 632, save_map);
				drawTexture(renderer, 1108, 709, back_to_map);
				if (material == 5 || material == 4 || material == 6) drawTexture(renderer, 1048, 394, map_edit_earthquake);
				if (back_to_map_button == true) stretchTextureEx(renderer, 1081, 708, 27, 27, arrow, 0);
				else if (map_editor_save_button == true) drawTexture(renderer,1060, 623,arrow );

/*				for (i = 0; i < population; i++) for (j = 0; j < object[i].shape->vertex_num; j++) {
					float vertex[2];
					vertex[0] = object[i].x[0] + object[i].shape->vertex[j][0];
					vertex[1] = object[i].x[1] + object[i].shape->vertex[j][1];
					stretchTextureEx(renderer, vertex[0] * trans - 2, vertex[1] * trans - 2, 5, 5, reddot, 0);
				}*/

				if (material == 5 || material == 4 || material == 6) {
					if (amplitude != 0.1f)stretchTextureEx(renderer, 1216, 440, 22, 28, small_arrow, 180);
					sprintf(text_string, "%.1f", amplitude);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1255, 435, text);
					drawTexture(renderer, 1255, 435, text);
					drawTexture(renderer, 1255, 435, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
					if (amplitude != 5.0f)stretchTextureEx(renderer, 1320, 440, 22, 28, small_arrow, 0);
				}

				if (material == 5 || material == 4 || material == 6) {
					if(earthquake_type != 0)stretchTextureEx(renderer, 1176, 485, 22, 28, small_arrow, 180);
					if (earthquake_type == 0) {
						sprintf(text_string, "x");
						text_surface = TTF_RenderText_Blended(font30, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 1255, 478, text);
						drawTexture(renderer, 1255, 478, text);
						drawTexture(renderer, 1255, 478, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);
					}
					else if (earthquake_type == 1) {
						drawTexture(renderer, 1216, 492, LR_img);
						sprintf(text_string, "cos");
						text_surface = TTF_RenderText_Blended(font30, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 1256, 478, text);
						drawTexture(renderer, 1256, 478, text);
						drawTexture(renderer, 1256, 478, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);
					}
					else if (earthquake_type == 2) {
						drawTexture(renderer, 1216, 492, LR_img);
						sprintf(text_string, "sin");
						text_surface = TTF_RenderText_Blended(font30, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 1256, 478, text);
						drawTexture(renderer, 1256, 478, text);
						drawTexture(renderer, 1256, 478, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);
					}
					else if (earthquake_type == 3) {
						drawTexture(renderer, 1226, 485, UD_img);
						sprintf(text_string, "cos");
						text_surface = TTF_RenderText_Blended(font30, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 1256, 478, text);
						drawTexture(renderer, 1256, 478, text);
						drawTexture(renderer, 1256, 478, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);
					}
					else if (earthquake_type == 4) {
						drawTexture(renderer, 1226, 485, UD_img);
						sprintf(text_string, "sin");
						text_surface = TTF_RenderText_Blended(font30, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 1256, 478, text);
						drawTexture(renderer, 1256, 478, text);
						drawTexture(renderer, 1256, 478, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);
					}
					else if (earthquake_type == 5) drawTexture(renderer, 1250, 485, UDLR_img);
					else if (earthquake_type == 6) drawTexture(renderer, 1245, 490, rotation_img);
					else if (earthquake_type == 7) drawTexture(renderer, 1245, 485, circle_img);
					else if (earthquake_type == 8) drawTexture(renderer, 1250, 495, swing_img);
					if(earthquake_type != 8)stretchTextureEx(renderer, 1320, 485, 22, 28, small_arrow, 0);
				}

				if (material == 5 || material == 4 || material == 6) {
					if(time_duration != 1.5f)stretchTextureEx(renderer, 1203, 535, 22, 28, small_arrow, 180);
					sprintf(text_string, "%.1fs", time_duration - 1.0f);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1240, 527, text);
					drawTexture(renderer, 1240, 527, text);
					drawTexture(renderer, 1240, 527, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
					if (time_duration != 10.5f)stretchTextureEx(renderer, 1320, 535, 22, 28, small_arrow, 0);
				}

				if (material == 5 || material == 4 || material == 6) {
				if(frequency != 0.1f)stretchTextureEx(renderer, 1216, 585, 22, 28, small_arrow, 180);
				sprintf(text_string, "%.1f",frequency);
				text_surface = TTF_RenderText_Blended(font30, text_string, white);
				text = SDL_CreateTextureFromSurface(renderer, text_surface);
				drawTexture(renderer, 1255, 578, text);
				drawTexture(renderer, 1255, 578, text);
				drawTexture(renderer, 1255, 578, text);
				SDL_FreeSurface(text_surface);
				SDL_DestroyTexture(text);
				if (frequency != 2.0f)stretchTextureEx(renderer, 1320, 585, 22, 28, small_arrow, 0);
				}

				if (material == 7) {
					sprintf(text_string, "Budget");
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1050, 450, text);
					drawTexture(renderer, 1050, 450, text);
					drawTexture(renderer, 1050, 450, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);

					if(budget >0)stretchTextureEx(renderer, 1179, 485, 22, 28, small_arrow, 90);
					sprintf(text_string, "%d",hundreds);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1180, 450, text);
					drawTexture(renderer, 1180, 450, text);
					drawTexture(renderer, 1180, 450, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
					if(budget<999)stretchTextureEx(renderer, 1179, 430, 22, 28, small_arrow, 270);

					if (budget >0)stretchTextureEx(renderer, 1219, 485, 22, 28, small_arrow, 90);
					sprintf(text_string, "%d",tenth);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1220, 450, text);
					drawTexture(renderer, 1220, 450, text);
					drawTexture(renderer, 1220, 450, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
					if (budget<999)stretchTextureEx(renderer, 1219, 430, 22, 28, small_arrow, 270);

					if (budget >0)stretchTextureEx(renderer, 1259, 485, 22, 28, small_arrow, 90);
					sprintf(text_string, "%d",units);
					text_surface = TTF_RenderText_Blended(font30, text_string, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1260, 450, text);
					drawTexture(renderer, 1260, 450, text);
					drawTexture(renderer, 1260, 450, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
					if (budget<999)stretchTextureEx(renderer, 1259, 430, 22, 28, small_arrow, 270);

					char map_text[40];
					sprintf(map_text, "place cursor on stones");
					text_surface = TTF_RenderText_Blended(font25, map_text, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1050, 510, text);
					drawTexture(renderer, 1050, 510, text);
					drawTexture(renderer, 1050, 510, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
					sprintf(map_text, "and press 'v' or 'b'");
					text_surface = TTF_RenderText_Blended(font25, map_text, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1050, 540, text);
					drawTexture(renderer, 1050, 540, text);
					drawTexture(renderer, 1050, 540, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
					sprintf(map_text, "to manipulate velocity");
					text_surface = TTF_RenderText_Blended(font25, map_text, white);
					text = SDL_CreateTextureFromSurface(renderer, text_surface);
					drawTexture(renderer, 1050, 570, text);
					drawTexture(renderer, 1050, 570, text);
					drawTexture(renderer, 1050, 570, text);
					SDL_FreeSurface(text_surface);
					SDL_DestroyTexture(text);
				}

					if (material == 5) {
						stretchTextureEx(renderer, 1124, 100, Length*trans, Length*trans, pre_wood_one_block, 0);
						stretchTextureEx(renderer, 1234, 100, 2 * Length*trans, Length*trans, pre_wood_two_block, 0);
						stretchTextureEx(renderer, 1084, 221, 3 * Length*trans, Length*trans, pre_wood_three_block, 0);
						stretchTextureEx_revise(renderer, 1235, 192, 2 * Length*trans, Length*trans*sqrt(3) + 1, Length*trans, Length*trans*sqrt(3) / 2.0, pre_wood_hexagon_block, 0);
						stretchTextureEx_revise(renderer, 1104, 308, 2 * Length*trans, 2 * Length*trans, 19 * Length*trans / 21.0, 23 * Length*trans / 21.0, pre_wood_polygon1_block, 0);
						stretchTextureEx_revise(renderer, 1235, 347, 2 * Length*trans, sqrt(3)*Length*trans / 2.0, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_wood_trapezoid_block, 0);
					}
					else if (material == 6) {
						stretchTextureEx(renderer, 1124, 100, Length*trans, Length*trans, pre_metal_one_block, 0);
						stretchTextureEx(renderer, 1234, 100, 2 * Length*trans, Length*trans, pre_metal_two_block, 0);
						stretchTextureEx(renderer, 1084, 221, 3 * Length*trans, Length*trans, pre_metal_three_block, 0);
						stretchTextureEx_revise(renderer, 1235, 192, 2 * Length*trans, Length*trans*sqrt(3) + 1, Length*trans, Length*trans*sqrt(3) / 2.0, pre_metal_hexagon_block, 0);
						stretchTextureEx_revise(renderer, 1104, 308, 2 * Length*trans, 2 * Length*trans, 19 * Length*trans / 21.0, 23 * Length*trans / 21.0, pre_metal_polygon1_block, 0);
						stretchTextureEx_revise(renderer, 1235, 347, 2 * Length*trans, sqrt(3)*Length*trans / 2.0, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_metal_trapezoid_block, 0);
					}
					else if (material == 4) {
						stretchTextureEx(renderer, 1124, 100, Length*trans, Length*trans, pre_rock_one_block, 0);
						stretchTextureEx(renderer, 1234, 100, 2 * Length*trans, Length*trans, pre_rock_two_block, 0);
						stretchTextureEx(renderer, 1084, 221, 3 * Length*trans, Length*trans, pre_rock_three_block, 0);
						stretchTextureEx_revise(renderer, 1235, 192, 2 * Length*trans, Length*trans*sqrt(3) + 1, Length*trans, Length*trans*sqrt(3) / 2.0, pre_rock_hexagon_block, 0);
						stretchTextureEx_revise(renderer, 1104, 308, 2 * Length*trans, 2 * Length*trans, 19 * Length*trans / 21.0, 23 * Length*trans / 21.0, pre_rock_polygon1_block, 0);
						stretchTextureEx_revise(renderer, 1235, 347, 2 * Length*trans, sqrt(3)*Length*trans / 2.0, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_rock_trapezoid_block, 0);
					}
					else if (material == 7) {
						drawTexture(renderer, 1110, 80, bombimg);
						drawTexture(renderer, 1094, 211, stone);
						drawTexture(renderer, 1114, 327, cat);
						stretchTextureEx(renderer, 1224, 110, 100, 28, large_soil, 0);
						stretchTextureEx(renderer, 1224, 221, 100, 37, ground_cone_small, 0);
						stretchTextureEx(renderer, 1224, 305, 100, 112, hill, 0);
					}

					if (click_one_block == true) {
						if (material == 5) stretchTextureEx(renderer, mouse_x - Length*trans / 2, mouse_y - Length*trans / 2, Length*trans, Length*trans, pre_wood_one_block, angle);
						else if (material == 6) stretchTextureEx(renderer, mouse_x - Length*trans / 2, mouse_y - Length*trans / 2, Length*trans, Length*trans, pre_metal_one_block, angle);
						else if (material == 4) stretchTextureEx(renderer, mouse_x - Length*trans / 2, mouse_y - Length*trans / 2, Length*trans, Length*trans, pre_rock_one_block, angle);
						else if (material == 7) stretchTextureEx(renderer, mouse_x - 35, mouse_y - 45, 70, 70, bombimg, 0);
					}
					else if (click_two_block == true) {
						if (material == 5) stretchTextureEx(renderer, mouse_x - Length*trans, mouse_y - Length*trans / 2, 2 * Length*trans, Length*trans, pre_wood_two_block, angle);
						else if (material == 6) stretchTextureEx(renderer, mouse_x - Length*trans, mouse_y - Length*trans / 2, 2 * Length*trans, Length*trans, pre_metal_two_block, angle);
						else if (material == 4) stretchTextureEx(renderer, mouse_x - Length*trans, mouse_y - Length*trans / 2, 2 * Length*trans, Length*trans, pre_rock_two_block, angle);
						else if (material == 7)stretchTextureEx(renderer, mouse_x - 7 * Length*trans, mouse_y - 2 * Length*trans, 14 * Length*trans, 4 * Length*trans, large_soil, angle);
					}
					else if (click_three_block == true) {
						if (material == 5) stretchTextureEx(renderer, mouse_x - 3 * Length*trans / 2, mouse_y - Length*trans / 2, 3 * Length*trans, Length*trans, pre_wood_three_block, angle);
						else if (material == 6) stretchTextureEx(renderer, mouse_x - 3 * Length*trans / 2, mouse_y - Length*trans / 2, 3 * Length*trans, Length*trans, pre_metal_three_block, angle);
						else if (material == 4) stretchTextureEx(renderer, mouse_x - 3 * Length*trans / 2, mouse_y - Length*trans / 2, 3 * Length*trans, Length*trans, pre_rock_three_block, angle);
						else if(material == 7) stretchTextureEx(renderer, mouse_x - 49.96*Length*trans / 40.0, mouse_y - 33.02*Length*trans / 40.0, 101 * Length*trans / 40.0, 69.0*Length*trans / 40.0, stone, angle);
					}
					else if (click_hexagon_block == true) {
						if (material == 5) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_wood_hexagon_block, angle);
						else if (material == 6) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_metal_hexagon_block, angle);
						else if (material == 4) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_rock_hexagon_block, angle);
						else if(material == 7) stretchTextureEx_revise(renderer, mouse_x - 77.5*Length*trans / 40.0, mouse_y - 4459.0*Length*trans / (219.0*40.0), 155.0*Length*trans / 40.0, 58.0*Length*trans / 40.0, 77.5*Length*trans / 40.0, 20.36*Length*trans / 40.0, ground_cone_small, angle);
					}
					else if (click_polygon1_block == true) {
						if (material == 5) stretchTextureEx_revise(renderer, mouse_x - 19.0*Length*trans / 21.0, mouse_y - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_wood_polygon1_block, angle);
						else if (material == 6) stretchTextureEx_revise(renderer, mouse_x - 19.0*Length*trans / 21.0, mouse_y - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_metal_polygon1_block, angle);
						else if (material == 4) stretchTextureEx_revise(renderer, mouse_x - 19.0*Length*trans / 21.0, mouse_y - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_rock_polygon1_block, angle);
						else if (material == 7) stretchTextureEx_revise(renderer, mouse_x - 25.5*Length*trans / 40.0 - 3, mouse_y - 31.0*Length*trans / 40.0, 51 * Length*trans / 40.0, 62.0*Length*trans / 40.0, 0.733511*Length*trans, 0.759908*Length*trans, cat, angle);
					}
					else if (click_trapezoid_block == true) {
						if (material == 5) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_wood_trapezoid_block, angle);
						else if (material == 6) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_metal_trapezoid_block, angle);
						else if (material == 4) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_rock_trapezoid_block, angle);
						else if (material == 7) stretchTextureEx_revise(renderer, mouse_x - 2.946671f*Length*trans, mouse_y - 3.543418f*Length*trans, 250.0*Length*trans / 40.0, 279.0*Length*trans / 40.0, 117.809f*Length*trans / 40.0, 142.2465f*Length*trans / 40.0f, hill, angle);
					}

					for (i = 0; i < population; i++) {
						if (object[i].shape->number == 1) {
							if (object[i].material.number == 2)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, wood_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, metal_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, rock_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, pre_wood_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, pre_metal_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, pre_rock_one_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 2) {
							if (object[i].material.number == 2) stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, wood_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, metal_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, rock_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, pre_wood_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, pre_metal_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, pre_rock_two_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 3) {
							if (object[i].material.number == 2) stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, wood_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, metal_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, rock_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, pre_wood_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, pre_metal_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, pre_rock_three_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 6) {
							if (object[i].material.number == 2) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), wood_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), metal_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), rock_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_wood_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_metal_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_rock_trapezoid_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 8) {
							if (object[i].material.number == 2) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, wood_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, metal_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, rock_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_wood_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_metal_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_rock_hexagon_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 9) {
							if (object[i].material.number == 2) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, wood_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, metal_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, rock_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_wood_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_metal_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_rock_polygon1_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 80) {
							stretchTextureEx(renderer, object[i].x[0] * trans - 49.96*Length*trans / 40.0, object[i].x[1] * trans - 33.02*Length*trans / 40.0, 101 * Length*trans / 40.0, 69.0*Length*trans / 40.0, stone, object[i].th * 180 / pi);
							if (!(object[i].v[0] == 0 && object[i].v[1] == 0)) {
								float x, y, theta, mag;
								x = object[i].v[0]; y = object[i].v[1];
								mag = sqrt(x*x + y*y);
								x = x / mag; y = y / mag;
								theta = asin(absolute(y));
								if (x >= 0 && y >= 0) theta = theta;
								else if (x >= 0 && y < 0) theta = 2 * pi - theta;
								else if (x < 0 && y >= 0) theta = pi - theta;
								else if (x < 0 && y < 0) theta = pi + theta;
								stretchTextureEx(renderer, object[i].x[0] * trans + x * 100 - 25, object[i].x[1] * trans + y * 100, 50, 21, velocity_arrow, theta * 180 / pi);
							}
						}
						else if (object[i].shape->number == 81) { //0.733511 0.759908
							stretchTextureEx_revise(renderer, object[i].x[0] * trans - 25.5*Length*trans / 40.0 - 3, object[i].x[1] * trans - 31.0*Length*trans / 40.0, 51 * Length*trans / 40.0, 62.0*Length*trans / 40.0, 0.733511*Length*trans, 0.759908*Length*trans, cat, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 97) {
							stretchTextureEx_revise(renderer, object[i].x[0] * trans - 2.946671f*Length*trans, object[i].x[1] * trans - 3.543418f*Length*trans, 250.0*Length*trans / 40.0, 279.0*Length*trans / 40.0, 117.809f*Length*trans / 40.0, 142.2465f*Length*trans / 40.0f, hill, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 98) {
							stretchTextureEx_revise(renderer, object[i].x[0] * trans - 77.5*Length*trans / 40.0, object[i].x[1] * trans - 4459.0*Length*trans / (219.0*40.0), 155.0*Length*trans / 40.0, 58.0*Length*trans / 40.0, 77.5*Length*trans / 40.0, 20.36*Length*trans / 40.0, ground_cone_small, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 99) {
							stretchTextureEx(renderer, object[i].x[0] * trans - 7 * Length*trans, object[i].x[1] * trans - 2 * Length*trans, 14 * Length*trans, 4 * Length*trans, large_soil, object[i].th * 180 / pi);
						}
					}

					for (i = 0; i < bomb_population; i++) {
						stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0);
					}
					
					if (save_screen == true) {
						underbar_time = underbar_time + (1 / fpss)*rendering_factor;

						drawTexture(renderer, 230, 207, running_end_menu);

						sprintf(text_string, "map name");
						text_surface = TTF_RenderText_Blended(font40, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 300, 250, text);
						drawTexture(renderer, 300, 250, text);
						drawTexture(renderer, 300, 250, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);

						drawTexture(renderer, 300, 330, small_arrow);

						text_surface = TTF_RenderText_Blended(font40, map_name, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 340, 315, text);
						drawTexture(renderer, 340, 315, text);
						drawTexture(renderer, 340, 315, text);

						if (underbar_time < 2.0 && map_name_length<15) {
								int w, h;
								SDL_QueryTexture(text, NULL, NULL, &w, &h);
								if(map_name_length>1) drawTexture(renderer, 340 + w + 5, 330, input_bar);
								else drawTexture(renderer, 340 + 5, 330, input_bar);
						}
						else if (underbar_time > 4.0)underbar_time = 0;

						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);

						sprintf(text_string, "save");
						text_surface = TTF_RenderText_Blended(font70, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 310, 450, text);
						drawTexture(renderer, 310, 450, text);
						drawTexture(renderer, 310, 450, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);
						if (save_mouse == true) drawTexture(renderer, 245, 460, arrow); //x > 590 && x <= 513 && y > 450 && y <= 531

						sprintf(text_string, "cancel");
						text_surface = TTF_RenderText_Blended(font70, text_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 590, 450, text);
						drawTexture(renderer, 590, 450, text);
						drawTexture(renderer, 590, 450, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);
						if (cancle_mouse == true) drawTexture(renderer, 525, 460, arrow); //x > 590 && x <= 513 && y > 450 && y <= 531
					}

				if (next_click == false && escape_map_editor == false && map_editing == true) SDL_RenderPresent(renderer);
				total_frame_end = SDL_GetPerformanceCounter();
				total_time = (float)(total_frame_end - total_frame_start) / ((float)SDL_GetPerformanceFrequency());
				total_delay_time = (int)(((1 / fpss)*rendering_factor - total_time) * 1000);

				if (total_delay_time>0) SDL_Delay(total_delay_time);
				else SDL_Delay(1);
				SDL_RenderClear(renderer);
			}

			while (quit == false && scene == map_edit &&map_running == true) {

				Object * object = NULL;
				Object * tempt_object;
				cinf * contact;

				Bomb * bomb = NULL;
				bool explo = false;
				int bomb_population = 0;

				float angle = 0;
				int population = 0;
				int material = 2; // wood
				int click_x, click_y;
				bool stage_prepare = true;
				bool stage_start = false;
				bool stage_start_button = false;
				bool back_to_stage_button = false;
				bool click_one_block = false;
				bool click_two_block = false;
				bool click_three_block = false;
				bool click_hexagon_block = false;
				bool click_polygon1_block = false;
				bool click_trapezoid_block = false;
				bool copy = false;
				char click_block = -1; // index of block

				next_click = false;

				int i, j, k;

				map = loadTexture("resource\\editor_back.png");
				right_menu = loadTexture("resource\\stage_menu.png");
				back_to_stage = loadTexture("resource\\back_to_stage.png");
				description = loadTexture("resource\\running_description.png");
				start = loadTexture("resource\\start!.png");
				wood_one_block = loadTexture("resource\\wood_one.png");
				wood_two_block = loadTexture("resource\\wood_two.png");
				wood_three_block = loadTexture("resource\\wood_three.png");
				wood_hexagon_block = loadTexture("resource\\wood_hexagon.png");
				wood_polygon1_block = loadTexture("resource\\wood_polygon1.png");
				wood_trapezoid_block = loadTexture("resource\\wood_trapezoid.png");
				pre_wood_one_block = loadTexture("resource\\pre_wood_one.png");
				pre_wood_two_block = loadTexture("resource\\pre_wood_two.png");
				pre_wood_three_block = loadTexture("resource\\pre_wood_three.png");
				pre_wood_hexagon_block = loadTexture("resource\\pre_wood_hexagon.png");
				pre_wood_polygon1_block = loadTexture("resource\\pre_wood_polygon1.png");
				pre_wood_trapezoid_block = loadTexture("resource\\pre_wood_trapezoid.png");
				metal_one_block = loadTexture("resource\\metal_one.png");
				metal_two_block = loadTexture("resource\\metal_two.png");
				metal_three_block = loadTexture("resource\\metal_three.png");
				metal_hexagon_block = loadTexture("resource\\metal_hexagon.png");
				metal_polygon1_block = loadTexture("resource\\metal_polygon1.png");
				metal_trapezoid_block = loadTexture("resource\\metal_trapezoid.png");
				pre_metal_one_block = loadTexture("resource\\pre_metal_one.png");
				pre_metal_two_block = loadTexture("resource\\pre_metal_two.png");
				pre_metal_three_block = loadTexture("resource\\pre_metal_three.png");
				pre_metal_hexagon_block = loadTexture("resource\\pre_metal_hexagon.png");
				pre_metal_polygon1_block = loadTexture("resource\\pre_metal_polygon1.png");
				pre_metal_trapezoid_block = loadTexture("resource\\pre_metal_trapezoid.png");
				rock_one_block = loadTexture("resource\\rock_one.png");
				rock_two_block = loadTexture("resource\\rock_two.png");
				rock_three_block = loadTexture("resource\\rock_three.png");
				rock_hexagon_block = loadTexture("resource\\rock_hexagon.png");
				rock_polygon1_block = loadTexture("resource\\rock_polygon1.png");
				rock_trapezoid_block = loadTexture("resource\\rock_trapezoid.png");
				pre_rock_one_block = loadTexture("resource\\pre_rock_one.png");
				pre_rock_two_block = loadTexture("resource\\pre_rock_two.png");
				pre_rock_three_block = loadTexture("resource\\pre_rock_three.png");
				pre_rock_hexagon_block = loadTexture("resource\\pre_rock_hexagon.png");
				pre_rock_polygon1_block = loadTexture("resource\\pre_rock_polygon1.png");
				pre_rock_trapezoid_block = loadTexture("resource\\pre_rock_trapezoid.png");
				explosion1 = loadTexture("resource\\explosion1.png");
				explosion2 = loadTexture("resource\\explosion2.png");
				explosion3 = loadTexture("resource\\explosion3.png");
				explosion4 = loadTexture("resource\\explosion4.png");
				explosion5 = loadTexture("resource\\explosion5.png");
				explosion6 = loadTexture("resource\\explosion6.png");
				explosion7 = loadTexture("resource\\explosion7.png");
				explosion8 = loadTexture("resource\\explosion8.png");
				explosion9 = loadTexture("resource\\explosion9.png");
				explosion10 = loadTexture("resource\\explosion10.png");
				explosion11 = loadTexture("resource\\explosion11.png");
				explosion12 = loadTexture("resource\\explosion12.png");
				explosion13 = loadTexture("resource\\explosion13.png");
				explosion14 = loadTexture("resource\\explosion14.png");
				explosion15 = loadTexture("resource\\explosion15.png");
				explosion16 = loadTexture("resource\\explosion16.png");
				explosion17 = loadTexture("resource\\explosion17.png");
				bombimg = loadTexture("resource\\bombimg.png");
				cat = loadTexture("resource\\cat.png");
				stone = loadTexture("resource\\stone.png");
				large_soil = loadTexture("resource\\two_soil.png");
				ground_cone_small = loadTexture("resource\\soil_cone_small.png");
				hill = loadTexture("resource\\hill.png");
				line = loadTexture("resource\\line.png");
				question_round = loadTexture("resource\\round_question.png");
				five$ = loadTexture("resource\\5$.png");
				seven$ = loadTexture("resource\\7$.png");
				ten$ = loadTexture("resource\\10$.png");
				thirteen$ = loadTexture("resource\\13$.png");
				fifteen$ = loadTexture("resource\\15$.png");
				eighteen$ = loadTexture("resource\\18$.png");
				twenty$ = loadTexture("resource\\20$.png");
				twentythree$ = loadTexture("resource\\23$.png");
				twentysix$ = loadTexture("resource\\26$.png");
				twentyseven$ = loadTexture("resource\\27$.png");
				thirty$ = loadTexture("resource\\30$.png");
				thirtysix$ = loadTexture("resource\\36$.png");
				redline = loadTexture("resource\\redline.png");
				velocity_arrow = loadTexture("resource\\velocity_arrow.png");
				star = loadTexture("resource\\star.png");
				blood_dot = loadTexture("resource\\blood_dot.png");
				stop_retry = loadTexture("resource\\stop_retry.png");
				pause = loadTexture("resource\\pause.png");
				normal_speed = loadTexture("resource\\normal_speed.png");
				x2speed = loadTexture("resource\\x2speed.png");
				speed_select = loadTexture("resource\\speed_select.png");
				running_end_menu = loadTexture("resource\\running_end_menu.png");
				back_to_home = loadTexture("resource\\back_to_home.png");
				next_arrow = loadTexture("resource\\next_arrow.png");
				re_arrow = loadTexture("resource\\re_arrow.png");

				float height;
				float timeduration;
				int earthquake_type;
				float amplitude;
				float frequency;
				float starttime;
				float endtime;
				int budget;
				int initial_budget;
				bool description_on = false;

				int total_frame_start;
				int total_frame_end;
				float total_time;
				int total_delay_time;

				/*================map edition================*/

				if (map1load == true || map2load == true || map3load == true) {
					FILE * load_file;
					switch (map_number) {
					case 1:
						load_file = fopen("map1.dat", "rb");

						free(map_name);
						fread(&map_name_length, sizeof(map_name_length), 1, load_file);
						map_name = (char *)malloc(sizeof(char)*map_name_length);
						for (i = 0; i < map_name_length; i++) fread(&map_name[i], sizeof(map_name[i]), 1, load_file);
						fread(&map1_year, sizeof(map1_year), 1, load_file);
						fread(&map1_month, sizeof(map1_month), 1, load_file);
						fread(&map1_day, sizeof(map1_day), 1, load_file);
						fread(&map1_hour, sizeof(map1_hour), 1, load_file);
						fread(&map1_min, sizeof(map1_min), 1, load_file);
						fread(&map1_sec, sizeof(map1_sec), 1, load_file);
						break;
					case 2:
						load_file = fopen("map2.dat", "rb");

						free(map_name);
						fread(&map_name_length, sizeof(map_name_length), 1, load_file);
						map_name = (char *)malloc(sizeof(char)*map_name_length);
						for (i = 0; i < map_name_length; i++) fread(&map_name[i], sizeof(map_name[i]), 1, load_file);
						fread(&map2_year, sizeof(map2_year), 1, load_file);
						fread(&map2_month, sizeof(map2_month), 1, load_file);
						fread(&map2_day, sizeof(map2_day), 1, load_file);
						fread(&map2_hour, sizeof(map2_hour), 1, load_file);
						fread(&map2_min, sizeof(map2_min), 1, load_file);
						fread(&map2_sec, sizeof(map2_sec), 1, load_file);
						break;
					case 3:
						load_file = fopen("map3.dat", "rb");

						free(map_name);
						fread(&map_name_length, sizeof(map_name_length), 1, load_file);
						map_name = (char *)malloc(sizeof(char)*map_name_length);
						for (i = 0; i < map_name_length; i++) fread(&map_name[i], sizeof(map_name[i]), 1, load_file);
						fread(&map3_year, sizeof(map3_year), 1, load_file);
						fread(&map3_month, sizeof(map3_month), 1, load_file);
						fread(&map3_day, sizeof(map3_day), 1, load_file);
						fread(&map3_hour, sizeof(map3_hour), 1, load_file);
						fread(&map3_min, sizeof(map3_min), 1, load_file);
						fread(&map3_sec, sizeof(map3_sec), 1, load_file);
						break;
					}
					fread(&population, sizeof(population), 1, load_file);
					object = (Object *)malloc(sizeof(Object)*population);
					for (i = 0; i < population; i++) {
						int shape_number;
						char material_number;
						float x1, x2, v1, v2, a1, a2, th, w, alp, F1, F2;
						fread(&shape_number, sizeof(shape_number), 1, load_file);
						fread(&material_number, sizeof(material_number), 1, load_file);
						fread(&x1, sizeof(x1), 1, load_file);
						fread(&x2, sizeof(x2), 1, load_file);
						fread(&v1, sizeof(v1), 1, load_file);
						fread(&v2, sizeof(v2), 1, load_file);
						fread(&a1, sizeof(a1), 1, load_file);
						fread(&a2, sizeof(a2), 1, load_file);
						fread(&th, sizeof(th), 1, load_file);
						fread(&w, sizeof(w), 1, load_file);
						fread(&alp, sizeof(alp), 1, load_file);
						fread(&F1, sizeof(F1), 1, load_file);
						fread(&F2, sizeof(F2), 1, load_file);
						set_object(&object[i], shape_number, material_number, x1*trans, x2*trans, v1*trans, v2*trans, a1*trans, a2*trans, th, w, alp, F1, F2);
					}
					fread(&budget, sizeof(budget), 1, load_file);
					print_digit(budget, &hundreds, &tenth, &units);
					fread(&height, sizeof(height), 1, load_file);
					fread(&timeduration, sizeof(timeduration), 1, load_file);
					fread(&earthquake_type, sizeof(earthquake_type), 1, load_file);
					fread(&amplitude, sizeof(amplitude), 1, load_file);
					fread(&frequency, sizeof(frequency), 1, load_file);
					fread(&bomb_population, sizeof(bomb_population), 1, load_file);
					bomb = (Bomb *)malloc(sizeof(Bomb)*bomb_population);
					for (i = 0; i < bomb_population; i++) {
						fread(&bomb[i].x[0], sizeof(bomb[i].x[0]), 1, load_file);
						fread(&bomb[i].x[1], sizeof(bomb[i].x[1]), 1, load_file);
						fread(&bomb[i].momentum, sizeof(bomb[i].momentum), 1, load_file);
					}
					map_load = false;
					fclose(load_file);
				}
				starttime = 1;
				endtime = timeduration + 1;
				if (bomb_population > 0) explo = false;
				else if (bomb_population == 0) explo = true;
				/*================map edition================*/
				initial_budget = budget;

				if (retry == true) { // if retry button was clicked
					if (population != 0) {
						for (i = 0; i < population; i++) {
							for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
							free(object[i].shape->vertex); free(object[i].shape->normal);
							free(object[i].shape->vertex_contact_index);
							free(object[i].shape->vertex_contact);
							free(object[i].shape);
						}
						free(object);
					}
					population = pre_population;
					budget = pre_budget;
					object = (Object *)malloc(sizeof(Object)*population);
					for (i = 0; i < population; i++) set_object(&object[i], pre_object[i].shape->number, pre_object[i].material.number, pre_object[i].x[0] * trans, pre_object[i].x[1] * trans, pre_object[i].v[0] * trans, pre_object[i].v[1] * trans, pre_object[i].a[0] * trans, pre_object[i].a[1] * trans, pre_object[i].th, pre_object[i].w, pre_object[i].alp, pre_object[i].F[0], pre_object[i].F[1]);
					if (population != 0) {
						for (i = 0; i < population; i++) {
							for (j = 0; j < pre_object[i].shape->vertex_num; j++) { free(pre_object[i].shape->vertex[j]); free(pre_object[i].shape->normal[j]); }
							free(pre_object[i].shape->vertex); free(pre_object[i].shape->normal);
							free(pre_object[i].shape->vertex_contact_index);
							free(pre_object[i].shape->vertex_contact);
							free(pre_object[i].shape);
						}
						free(pre_object);
					}
					retry = false;
				}

				for (i = 0; i < population; i++) reassign_vertex(&object[i]);

				while (stage_prepare == true && quit == false) {
					total_frame_start = SDL_GetPerformanceCounter();

					drawTexture(renderer, 0, 0, map);
					drawTexture(renderer, 0, height, line);
					drawTexture(renderer, 1004, 0, right_menu);

					if (description_on == false) {
						sprintf(budget_string, "budget  %d$", budget);
						text_surface = TTF_RenderText_Blended(font40, budget_string, white);
						text = SDL_CreateTextureFromSurface(renderer, text_surface);
						drawTexture(renderer, 1080, 530, text);
						drawTexture(renderer, 1080, 530, text);
						drawTexture(renderer, 1080, 530, text);
						SDL_FreeSurface(text_surface);
						SDL_DestroyTexture(text);
					}

					drawTexture(renderer, 1304, 40, question_round);
					if (description_on == false)drawTexture(renderer, 1108, 709, back_to_stage);
					if (description_on == true)drawTexture(renderer, 1043, 91, description);
					if (description_on == false) {
						drawTexture(renderer, 1088, 622, start);
						if (budget<0) drawTexture(renderer, 1088, 622, redline);
					}
					if (description_on == false) {
						if (material == 2) {
							stretchTextureEx(renderer, 1124, 100, Length*trans, Length*trans, wood_one_block, 0);
							drawTexture(renderer, 1124, 145, five$);
						}
						else if (material == 3) {
							stretchTextureEx(renderer, 1124, 100, Length*trans, Length*trans, metal_one_block, 0);
							drawTexture(renderer, 1114, 145, ten$);
						}
						else if (material == 1) {
							stretchTextureEx(renderer, 1124, 100, Length*trans, Length*trans, rock_one_block, 0);
							drawTexture(renderer, 1124, 145, seven$);
						}
					}
					if (description_on == false) {
						if (material == 2) {
							stretchTextureEx(renderer, 1234, 100, 2 * Length*trans, Length*trans, wood_two_block, 0);
							drawTexture(renderer, 1244, 145, ten$);
						}
						else if (material == 3) {
							stretchTextureEx(renderer, 1234, 100, 2 * Length*trans, Length*trans, metal_two_block, 0);
							drawTexture(renderer, 1244, 145, twenty$);
						}
						else if (material == 1) {
							stretchTextureEx(renderer, 1234, 100, 2 * Length*trans, Length*trans, rock_two_block, 0);
							drawTexture(renderer, 1244, 145, fifteen$);
						}
					}
					if (description_on == false) {
						if (material == 2) {
							stretchTextureEx(renderer, 1084, 221, 3 * Length*trans, Length*trans, wood_three_block, 0);
							drawTexture(renderer, 1114, 266, fifteen$);
						}
						else if (material == 3) {
							stretchTextureEx(renderer, 1084, 221, 3 * Length*trans, Length*trans, metal_three_block, 0);
							drawTexture(renderer, 1114, 266, thirty$);
						}
						else if (material == 1) {
							stretchTextureEx(renderer, 1084, 221, 3 * Length*trans, Length*trans, rock_three_block, 0);
							drawTexture(renderer, 1114, 266, twentythree$);
						}
					}
					if (description_on == false) {
						if (material == 2) {
							stretchTextureEx_revise(renderer, 1235, 192, 2 * Length*trans, Length*trans*sqrt(3) + 1, Length*trans, Length*trans*sqrt(3) / 2.0, wood_hexagon_block, 0);
							drawTexture(renderer, 1244, 266, thirteen$);
						}
						else if (material == 3) {
							stretchTextureEx_revise(renderer, 1235, 192, 2 * Length*trans, Length*trans*sqrt(3) + 1, Length*trans, Length*trans*sqrt(3) / 2.0, metal_hexagon_block, 0);
							drawTexture(renderer, 1244, 266, twentysix$);
						}
						else if (material == 1) {
							stretchTextureEx_revise(renderer, 1235, 192, 2 * Length*trans, Length*trans*sqrt(3) + 1, Length*trans, Length*trans*sqrt(3) / 2.0, rock_hexagon_block, 0);
							drawTexture(renderer, 1244, 266, twenty$);
						}
					}
					if (description_on == false) {
						if (material == 2) {
							stretchTextureEx_revise(renderer, 1104, 308, 2 * Length*trans, 2 * Length*trans, 19 * Length*trans / 21.0, 23 * Length*trans / 21.0, wood_polygon1_block, 0);
							drawTexture(renderer, 1114, 387, eighteen$);
						}
						else if (material == 3) {
							stretchTextureEx_revise(renderer, 1104, 308, 2 * Length*trans, 2 * Length*trans, 19 * Length*trans / 21.0, 23 * Length*trans / 21.0, metal_polygon1_block, 0);
							drawTexture(renderer, 1114, 387, thirtysix$);
						}
						else if (material == 1) {
							stretchTextureEx_revise(renderer, 1104, 308, 2 * Length*trans, 2 * Length*trans, 19 * Length*trans / 21.0, 23 * Length*trans / 21.0, rock_polygon1_block, 0);
							drawTexture(renderer, 1114, 387, twentyseven$);
						}
					}
					if (description_on == false) {
						if (material == 2) {
							stretchTextureEx_revise(renderer, 1235, 347, 2 * Length*trans, sqrt(3)*Length*trans / 2.0, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), wood_trapezoid_block, 0);
							drawTexture(renderer, 1245, 387, ten$);
						}
						else if (material == 3) {
							stretchTextureEx_revise(renderer, 1235, 347, 2 * Length*trans, sqrt(3)*Length*trans / 2.0, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), metal_trapezoid_block, 0);
							drawTexture(renderer, 1245, 387, twenty$);
						}
						else if (material == 1) {
							stretchTextureEx_revise(renderer, 1235, 347, 2 * Length*trans, sqrt(3)*Length*trans / 2.0, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), rock_trapezoid_block, 0);
							drawTexture(renderer, 1245, 387, fifteen$);
						}
					}

					int x, y;
					x = event.motion.x;
					y = event.motion.y;

					int mouse_x, mouse_y;
					/*				for (i = 0; i < population; i++) for (j = 0; j < object[i].shape->vertex_num; j++) {
					float vertex[2];
					vertex[0] = object[i].x[0] + object[i].shape->vertex[j][0];
					vertex[1] = object[i].x[1] + object[i].shape->vertex[j][1];
					stretchTextureEx(renderer, vertex[0] * trans - 2, vertex[1] * trans - 2, 5, 5, reddot, 0);
					}*/

					while (SDL_PollEvent(&event)) {
						if (event.type == SDL_QUIT) quit = true;

						if (event.type == SDL_MOUSEBUTTONDOWN) {
							if (event.button.button == SDL_BUTTON_LEFT) {
								int x, y;
								x = event.motion.x;
								y = event.motion.y;
								if (x >= 1108 && x <= 1355 && y >= 709 && y <= 736) { // click back to stage select
									stage_prepare = false;
									SDL_DestroyTexture(map);
									SDL_DestroyTexture(back_to_stage);
									SDL_DestroyTexture(description);
									SDL_DestroyTexture(start);
									SDL_DestroyTexture(right_menu);
									SDL_DestroyTexture(wood_one_block);
									SDL_DestroyTexture(wood_two_block);
									SDL_DestroyTexture(wood_three_block);
									SDL_DestroyTexture(wood_hexagon_block);
									SDL_DestroyTexture(wood_polygon1_block);
									SDL_DestroyTexture(wood_trapezoid_block);
									SDL_DestroyTexture(pre_wood_one_block);
									SDL_DestroyTexture(pre_wood_two_block);
									SDL_DestroyTexture(pre_wood_three_block);
									SDL_DestroyTexture(pre_wood_hexagon_block);
									SDL_DestroyTexture(pre_wood_polygon1_block);
									SDL_DestroyTexture(pre_wood_trapezoid_block);
									SDL_DestroyTexture(metal_one_block);
									SDL_DestroyTexture(metal_two_block);
									SDL_DestroyTexture(metal_three_block);
									SDL_DestroyTexture(metal_hexagon_block);
									SDL_DestroyTexture(metal_polygon1_block);
									SDL_DestroyTexture(metal_trapezoid_block);
									SDL_DestroyTexture(pre_metal_one_block);
									SDL_DestroyTexture(pre_metal_two_block);
									SDL_DestroyTexture(pre_metal_three_block);
									SDL_DestroyTexture(pre_metal_hexagon_block);
									SDL_DestroyTexture(pre_metal_polygon1_block);
									SDL_DestroyTexture(pre_metal_trapezoid_block);
									SDL_DestroyTexture(rock_one_block);
									SDL_DestroyTexture(rock_two_block);
									SDL_DestroyTexture(rock_three_block);
									SDL_DestroyTexture(rock_hexagon_block);
									SDL_DestroyTexture(rock_polygon1_block);
									SDL_DestroyTexture(rock_trapezoid_block);
									SDL_DestroyTexture(pre_rock_one_block);
									SDL_DestroyTexture(pre_rock_two_block);
									SDL_DestroyTexture(pre_rock_three_block);
									SDL_DestroyTexture(pre_rock_hexagon_block);
									SDL_DestroyTexture(pre_rock_polygon1_block);
									SDL_DestroyTexture(pre_rock_trapezoid_block);
									SDL_DestroyTexture(explosion1);
									SDL_DestroyTexture(explosion2);
									SDL_DestroyTexture(explosion3);
									SDL_DestroyTexture(explosion4);
									SDL_DestroyTexture(explosion5);
									SDL_DestroyTexture(explosion6);
									SDL_DestroyTexture(explosion7);
									SDL_DestroyTexture(explosion8);
									SDL_DestroyTexture(explosion9);
									SDL_DestroyTexture(explosion11);
									SDL_DestroyTexture(explosion10);
									SDL_DestroyTexture(explosion12);
									SDL_DestroyTexture(explosion13);
									SDL_DestroyTexture(explosion14);
									SDL_DestroyTexture(explosion15);
									SDL_DestroyTexture(explosion16);
									SDL_DestroyTexture(explosion17);
									SDL_DestroyTexture(bombimg);
									SDL_DestroyTexture(cat);
									SDL_DestroyTexture(stone);
									SDL_DestroyTexture(large_soil);
									SDL_DestroyTexture(ground_cone_small);
									SDL_DestroyTexture(hill);
									SDL_DestroyTexture(line);
									SDL_DestroyTexture(question_round);
									SDL_DestroyTexture(five$);
									SDL_DestroyTexture(seven$);
									SDL_DestroyTexture(ten$);
									SDL_DestroyTexture(thirteen$);
									SDL_DestroyTexture(fifteen$);
									SDL_DestroyTexture(eighteen$);
									SDL_DestroyTexture(twenty$);
									SDL_DestroyTexture(twentythree$);
									SDL_DestroyTexture(twentysix$);
									SDL_DestroyTexture(twentyseven$);
									SDL_DestroyTexture(thirty$);
									SDL_DestroyTexture(thirtysix$);
									SDL_DestroyTexture(redline);
									SDL_DestroyTexture(velocity_arrow);
									SDL_DestroyTexture(star);
									SDL_DestroyTexture(blood_dot);
									SDL_DestroyTexture(stop_retry);
									SDL_DestroyTexture(pause);
									SDL_DestroyTexture(normal_speed);
									SDL_DestroyTexture(x2speed);
									SDL_DestroyTexture(speed_select);
									SDL_DestroyTexture(running_end_menu);
									SDL_DestroyTexture(back_to_home);
									SDL_DestroyTexture(next_arrow);
									SDL_DestroyTexture(re_arrow);
									map_running = false;
									if (population != 0) {
										for (i = 0; i < population; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
											free(object[i].shape->vertex); free(object[i].shape->normal);
											free(object[i].shape->vertex_contact_index);
											free(object[i].shape->vertex_contact);
											free(object[i].shape);
										}
										free(object);
									}
									population = 0;
									if (bomb_population != 0) free(bomb);
									bomb_population = 0;
									Mix_PlayChannel(-1, click_sound, 0);
									Mix_HaltMusic();
									Mix_FadeInMusic(main_music, -1, 1000);
								}
								else if (x >= 1088 && x <= 1307 && y >= 622 && y <= 670 && budget >= 0) { // click start button
									stage_start = true;
									stage_prepare = false;
									SDL_DestroyTexture(back_to_stage);
									SDL_DestroyTexture(description);
									SDL_DestroyTexture(start);
									SDL_DestroyTexture(question_round);
									SDL_DestroyTexture(five$);
									SDL_DestroyTexture(seven$);
									SDL_DestroyTexture(ten$);
									SDL_DestroyTexture(thirteen$);
									SDL_DestroyTexture(fifteen$);
									SDL_DestroyTexture(eighteen$);
									SDL_DestroyTexture(twenty$);
									SDL_DestroyTexture(twentythree$);
									SDL_DestroyTexture(twentysix$);
									SDL_DestroyTexture(twentyseven$);
									SDL_DestroyTexture(thirty$);
									SDL_DestroyTexture(thirtysix$);
									SDL_DestroyTexture(redline);
									SDL_DestroyTexture(velocity_arrow);
									contact = (cinf *)malloc(sizeof(cinf) * 4 * population);
									for (i = 0; i < 4 * population; i++) {
										contact[i].run = false;
										contact[i].normal_impulse = 0;
										contact[i].tangent_impulse = 0;
									}
									pre_object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population; i++) set_object(&pre_object[i], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, object[i].a[0], object[i].a[1], object[i].th, object[i].w, object[i].alp, object[i].F[0], object[i].F[1]);
									pre_population = population;
									pre_budget = budget;
								}
								else if (x >= 1124 && x <= 1124 + Length*trans && y >= 100 && y <= 100 + Length*trans) { // click one block
									click_one_block = true;
								}
								else if (x >= 1234 && x <= 1234 + 2 * Length*trans && y >= 100 && y <= 100 + Length*trans) { // click two block
									click_two_block = true;
								}
								else if (x >= 1084 && x <= 1084 + 3 * Length*trans && y >= 216 && y <= 216 + Length*trans) { // click three block
									click_three_block = true;
								}
								else if (x >= 1235 && x <= 1235 + 2 * Length*trans && y >= 192 && y <= 192 + Length*trans*sqrt(3)) { // click hexagon block
									click_hexagon_block = true;
								}
								else if (x >= 1104 && x <= 1104 + 2 * Length*trans && y >= 308 && y <= 308 + 2 * Length*trans) { // click polygon1 block
									click_polygon1_block = true;
								}
								else if (x >= 1235 && x <= 1235 + 2 * Length*trans && y >= 347 && y <= 347 + Length*trans*sqrt(3) / 2.0) {
									click_trapezoid_block = true;
								}
								else {
									for (i = 0; i < population; i++) if (click_perception(mouse_x / trans, mouse_y / trans, &object[i]) < 0 && (object[i].material.number != 99 && object[i].material.number != 98 && object[i].material.number != 4 && object[i].material.number != 5 && object[i].material.number != 6 && object[i].material.number != 97)) {
										if (copy == true) {
											click_block = i; click_x = mouse_x; click_y = mouse_y;

											if (object[click_block].shape->number == 1) {
												if (object[click_block].material.number == 2) budget = budget - 5;
												else if (object[click_block].material.number == 3) budget = budget - 10;
												else if (object[click_block].material.number == 1) budget = budget - 7;
											}
											else if (object[click_block].shape->number == 2) {
												if (object[click_block].material.number == 2) budget = budget - 10;
												else if (object[click_block].material.number == 3) budget = budget - 20;
												else if (object[click_block].material.number == 1) budget = budget - 15;
											}
											else if (object[click_block].shape->number == 3) {
												if (object[click_block].material.number == 2) budget = budget - 15;
												else if (object[click_block].material.number == 3) budget = budget - 30;
												else if (object[click_block].material.number == 1) budget = budget - 23;
											}
											else if (object[click_block].shape->number == 4) {
												if (object[click_block].material.number == 2) budget = budget - 3;
												else if (object[click_block].material.number == 3) budget = budget - 6;
												else if (object[click_block].material.number == 1) budget = budget - 5;
											}
											else if (object[click_block].shape->number == 5) {
												if (object[click_block].material.number == 2) budget = budget - 3;
												else if (object[click_block].material.number == 3) budget = budget - 6;
												else if (object[click_block].material.number == 1) budget = budget - 5;
											}
											else if (object[click_block].shape->number == 6) {
												if (object[click_block].material.number == 2) budget = budget - 10;
												else if (object[click_block].material.number == 3) budget = budget - 20;
												else if (object[click_block].material.number == 1) budget = budget - 15;
											}
											else if (object[click_block].shape->number == 8) {
												if (object[click_block].material.number == 2) budget = budget - 13;
												else if (object[click_block].material.number == 3) budget = budget - 26;
												else if (object[click_block].material.number == 1) budget = budget - 20;
											}
											else if (object[click_block].shape->number == 9) {
												if (object[click_block].material.number == 2) budget = budget - 18;
												else if (object[click_block].material.number == 3) budget = budget - 36;
												else if (object[click_block].material.number == 1) budget = budget - 27;
											}

											tempt_object = (Object *)malloc(sizeof(Object)*population);
											for (i = 0; i < population; i++) set_object(&tempt_object[i], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
											if (population != 0) {
												for (i = 0; i < population; i++) {
													for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
													free(object[i].shape->vertex); free(object[i].shape->normal);
													free(object[i].shape->vertex_contact_index);
													free(object[i].shape->vertex_contact);
													free(object[i].shape);
												}
												free(object);
											}
											population = population + 1;
											object = (Object *)malloc(sizeof(Object)*population);
											for (i = 0; i < population - 1; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
											set_object(&object[population - 1], object[click_block].shape->number, object[click_block].material.number, object[click_block].x[0] * trans, object[click_block].x[1] * trans, 0, 0, 0, 0, object[click_block].th, 0, 0, 0, 0);
											if (population - 1 != 0) {
												for (i = 0; i < population - 1; i++) {
													for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
													free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
													free(tempt_object[i].shape->vertex_contact_index);
													free(tempt_object[i].shape->vertex_contact);
													free(tempt_object[i].shape);
												}
												free(tempt_object);
											}
											for (i = 0; i < population; i++) reassign_vertex(&object[i]);
										}
										else click_block = i; click_x = mouse_x; click_y = mouse_y;
									}
								}
							}
							else if (event.button.button == SDL_BUTTON_RIGHT) {
								for (k = 0; k < population; k++) if (click_perception(mouse_x / trans, mouse_y / trans, &object[k]) < 0 && (object[k].material.number != 99 && object[k].material.number != 98 && object[k].material.number != 4 && object[k].material.number != 5 && object[k].material.number != 6 && object[k].material.number != 97) && click_block == -1) {
									int i, j;

									if (object[k].shape->number == 1) {
										if (object[k].material.number == 2) budget = budget + 5;
										else if (object[k].material.number == 3) budget = budget + 10;
										else if (object[k].material.number == 1) budget = budget + 7;
									}
									else if (object[k].shape->number == 2) {
										if (object[k].material.number == 2) budget = budget + 10;
										else if (object[k].material.number == 3) budget = budget + 20;
										else if (object[k].material.number == 1) budget = budget + 15;
									}
									else if (object[k].shape->number == 3) {
										if (object[k].material.number == 2) budget = budget + 15;
										else if (object[k].material.number == 3) budget = budget + 30;
										else if (object[k].material.number == 1) budget = budget + 23;
									}
									else if (object[k].shape->number == 4) {
										if (object[k].material.number == 2) budget = budget + 3;
										else if (object[k].material.number == 3) budget = budget + 6;
										else if (object[k].material.number == 1) budget = budget + 5;
									}
									else if (object[k].shape->number == 5) {
										if (object[k].material.number == 2) budget = budget + 3;
										else if (object[k].material.number == 3) budget = budget + 6;
										else if (object[k].material.number == 1) budget = budget + 5;
									}
									else if (object[k].shape->number == 6) {
										if (object[k].material.number == 2) budget = budget + 10;
										else if (object[k].material.number == 3) budget = budget + 20;
										else if (object[k].material.number == 1) budget = budget + 15;
									}
									else if (object[k].shape->number == 8) {
										if (object[k].material.number == 2) budget = budget + 13;
										else if (object[k].material.number == 3) budget = budget + 26;
										else if (object[k].material.number == 1) budget = budget + 20;
									}
									else if (object[k].shape->number == 9) {
										if (object[k].material.number == 2) budget = budget + 18;
										else if (object[k].material.number == 3) budget = budget + 36;
										else if (object[k].material.number == 1) budget = budget + 27;
									}
									population = population - 1;
									tempt_object = (Object *)malloc(sizeof(Object)*population);
									i = 0; j = 0;
									for (i = 0; i < population + 1; i++) if (i != k) {
										set_object(&tempt_object[j], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
										j = j + 1;
									}
									for (i = 0; i < population + 1; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
										free(object[i].shape->vertex); free(object[i].shape->normal);
										free(object[i].shape->vertex_contact_index);
										free(object[i].shape->vertex_contact);
										free(object[i].shape);
									}
									free(object);
									object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
									for (i = 0; i < population; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
										free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
										free(tempt_object[i].shape->vertex_contact_index);
										free(tempt_object[i].shape->vertex_contact);
										free(tempt_object[i].shape);
									}
									free(tempt_object);
									for (i = 0; i < population; i++) reassign_vertex(&object[i]);
									Mix_PlayChannel(-1, erase, 0);
									break;
								}
							}
						}

						if (event.type == SDL_MOUSEMOTION) {
							int x, y;
							x = event.motion.x;
							y = event.motion.y;
							mouse_x = x;
							mouse_y = y;

							if (x >= 1108 && x <= 1355 && y >= 709 && y <= 736) { // on back to stage button
								back_to_stage_button = true;
							}
							else if (x >= 1304 && x <= 1341 && y >= 40 && y <= 77) { // on question round
								description_on = true;
							}
							else if (x >= 1088 && x <= 1307 && y >= 622 && y <= 670 && budget >= 0) { // on stage start button
								stage_start_button = true;
							}
							else {
								back_to_stage_button = false;
								description_on = false;
								stage_start_button = false;
							}
							if (click_block != -1) {
								object[click_block].x[0] = object[click_block].x[0] + (mouse_x - click_x) / trans;
								object[click_block].x[1] = object[click_block].x[1] + (mouse_y - click_y) / trans;
								click_x = mouse_x; click_y = mouse_y;
							}
						}

						if (event.type == SDL_MOUSEBUTTONUP) {
							if (click_one_block == true || click_two_block == true || click_three_block == true || click_hexagon_block == true || click_polygon1_block == true || click_trapezoid_block == true) {
								if (mouse_x < 1004) {
									int i, j;
									tempt_object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population; i++) set_object(&tempt_object[i], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
									if (population != 0) {
										for (i = 0; i < population; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
											free(object[i].shape->vertex); free(object[i].shape->normal);
											free(object[i].shape->vertex_contact_index);
											free(object[i].shape->vertex_contact);
											free(object[i].shape);
										}
										free(object);
									}
									population = population + 1;
									object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population - 1; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
									if (click_one_block == true) {
										click_one_block = false;
										set_object(&object[population - 1], 1, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 5;
										else if (material == 3) budget = budget - 10;
										else if (material == 1) budget = budget - 7;
									}
									else if (click_two_block == true) {
										click_two_block = false;
										set_object(&object[population - 1], 2, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 10;
										else if (material == 3) budget = budget - 20;
										else if (material == 1) budget = budget - 15;
									}
									else if (click_three_block == true) {
										click_three_block = false;
										set_object(&object[population - 1], 3, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 15;
										else if (material == 3) budget = budget - 30;
										else if (material == 1) budget = budget - 23;
									}
									else if (click_hexagon_block == true) {
										click_hexagon_block = false;
										set_object(&object[population - 1], 8, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 13;
										else if (material == 3) budget = budget - 26;
										else if (material == 1) budget = budget - 20;
									}
									else if (click_polygon1_block == true) {
										click_polygon1_block = false;
										set_object(&object[population - 1], 9, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 18;
										else if (material == 3) budget = budget - 36;
										else if (material == 1) budget = budget - 27;
									}
									else if (click_trapezoid_block == true) {
										click_trapezoid_block = false;
										set_object(&object[population - 1], 6, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
										if (material == 2) budget = budget - 10;
										else if (material == 3) budget = budget - 20;
										else if (material == 1) budget = budget - 15;
									}
									if (population - 1 != 0) {
										for (i = 0; i < population - 1; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
											free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
											free(tempt_object[i].shape->vertex_contact_index);
											free(tempt_object[i].shape->vertex_contact);
											free(tempt_object[i].shape);
										}
										free(tempt_object);
									}
									for (i = 0; i < population; i++) reassign_vertex(&object[i]);
									angle = 0;
									if (material == 2) Mix_PlayChannel(-1, hammering_wood, 0);
									else if (material == 3) Mix_PlayChannel(-1, hammering_iron, 0);
									else if (material == 1)Mix_PlayChannel(-1, hammering_brick, 0);
								}
								else if (mouse_x >= 1004) {
									click_one_block = false;
									click_two_block = false;
									click_three_block = false;
									click_hexagon_block = false;
									click_polygon1_block = false;
									click_trapezoid_block = false;
									angle = 0;
									Mix_PlayChannel(-1, erase, 0);
								}
							}
							if (click_block != -1) {
								if (object[click_block].x[0] >= 1004 / trans) {
									int i, j;

									if (object[click_block].shape->number == 1) {
										if (object[click_block].material.number == 2) budget = budget + 5;
										else if (object[click_block].material.number == 3) budget = budget + 10;
										else if (object[click_block].material.number == 1) budget = budget + 7;
									}
									else if (object[click_block].shape->number == 2) {
										if (object[click_block].material.number == 2) budget = budget + 10;
										else if (object[click_block].material.number == 3) budget = budget + 20;
										else if (object[click_block].material.number == 1) budget = budget + 15;
									}
									else if (object[click_block].shape->number == 3) {
										if (object[click_block].material.number == 2) budget = budget + 15;
										else if (object[click_block].material.number == 3) budget = budget + 30;
										else if (object[click_block].material.number == 1) budget = budget + 23;
									}
									else if (object[click_block].shape->number == 4) {
										if (object[click_block].material.number == 2) budget = budget + 3;
										else if (object[click_block].material.number == 3) budget = budget + 6;
										else if (object[click_block].material.number == 1) budget = budget + 5;
									}
									else if (object[click_block].shape->number == 5) {
										if (object[click_block].material.number == 2) budget = budget + 3;
										else if (object[click_block].material.number == 3) budget = budget + 6;
										else if (object[click_block].material.number == 1) budget = budget + 5;
									}
									else if (object[click_block].shape->number == 6) {
										if (object[click_block].material.number == 2) budget = budget + 10;
										else if (object[click_block].material.number == 3) budget = budget + 20;
										else if (object[click_block].material.number == 1) budget = budget + 15;
									}
									else if (object[click_block].shape->number == 8) {
										if (object[click_block].material.number == 2) budget = budget + 13;
										else if (object[click_block].material.number == 3) budget = budget + 26;
										else if (object[click_block].material.number == 1) budget = budget + 20;
									}
									else if (object[click_block].shape->number == 9) {
										if (object[click_block].material.number == 2) budget = budget + 18;
										else if (object[click_block].material.number == 3) budget = budget + 36;
										else if (object[click_block].material.number == 1) budget = budget + 27;
									}

									population = population - 1;
									tempt_object = (Object *)malloc(sizeof(Object)*population);
									i = 0; j = 0;
									for (i = 0; i < population + 1; i++) if (i != click_block) {
										set_object(&tempt_object[j], object[i].shape->number, object[i].material.number, object[i].x[0] * trans, object[i].x[1] * trans, object[i].v[0] * trans, object[i].v[1] * trans, 0, 0, object[i].th, object[i].w, 0, 0, 0);
										j = j + 1;
									}
									for (i = 0; i < population + 1; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
										free(object[i].shape->vertex); free(object[i].shape->normal);
										free(object[i].shape->vertex_contact_index);
										free(object[i].shape->vertex_contact);
										free(object[i].shape);
									}
									free(object);
									object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0] * trans, tempt_object[i].x[1] * trans, tempt_object[i].v[0] * trans, tempt_object[i].v[1] * trans, 0, 0, tempt_object[i].th, tempt_object[i].w, 0, 0, 0);
									for (i = 0; i < population; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
										free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
										free(tempt_object[i].shape->vertex_contact_index);
										free(tempt_object[i].shape->vertex_contact);
										free(tempt_object[i].shape);
									}
									free(tempt_object);
									Mix_PlayChannel(-1, erase, 0);
								}
								else {
									if (object[click_block].material.number == 2) Mix_PlayChannel(-1, hammering_wood, 0);
									else if (object[click_block].material.number == 3) Mix_PlayChannel(-1, hammering_iron, 0);
									else if (object[click_block].material.number == 1)Mix_PlayChannel(-1, hammering_brick, 0);
								}
								click_block = -1;
								for (i = 0; i < population; i++) reassign_vertex(&object[i]);
								angle = 0;
							}
						}

						if (event.type == SDL_KEYDOWN) {
							if (event.key.keysym.sym == SDLK_r) {
								if (click_one_block == true || click_two_block == true || click_three_block == true || click_hexagon_block == true || click_polygon1_block == true || click_trapezoid_block == true) { angle = angle - 15; Mix_PlayChannel(-1, gear, 0); }
								if (click_block != -1) { object[click_block].th = object[click_block].th - 15 * pi / 180; Mix_PlayChannel(-1, gear, 0); }
							}
							else if (event.key.keysym.sym == SDLK_t) {
								if (click_one_block == true || click_two_block == true || click_three_block == true || click_hexagon_block == true || click_polygon1_block == true || click_trapezoid_block == true) { angle = angle + 15; Mix_PlayChannel(-1, gear, 0); }
								if (click_block != -1) { object[click_block].th = object[click_block].th + 15 * pi / 180; Mix_PlayChannel(-1, gear, 0); }
							}
							else if (event.key.keysym.sym == SDLK_q) {
								material = material + 1;
								if (material == 4)material = 1;
							}
							else if (event.key.keysym.sym == SDLK_LCTRL) {
								copy = true;
							}
						}

						if (event.type == SDL_KEYUP) {
							if (event.key.keysym.sym == SDLK_LCTRL) {
								copy = false;
							}
						}
					}
					if (back_to_stage_button == true) stretchTextureEx(renderer, 1081, 708, 27, 27, arrow, 0);
					else if (stage_start_button == true) drawTexture(renderer, 1023, 613, arrow);

					if (click_one_block == true) {
						if (material == 2) stretchTextureEx(renderer, mouse_x - Length*trans / 2, mouse_y - Length*trans / 2, Length*trans, Length*trans, wood_one_block, angle);
						else if (material == 3) stretchTextureEx(renderer, mouse_x - Length*trans / 2, mouse_y - Length*trans / 2, Length*trans, Length*trans, metal_one_block, angle);
						else if (material == 1) stretchTextureEx(renderer, mouse_x - Length*trans / 2, mouse_y - Length*trans / 2, Length*trans, Length*trans, rock_one_block, angle);
					}
					else if (click_two_block == true) {
						if (material == 2) stretchTextureEx(renderer, mouse_x - Length*trans, mouse_y - Length*trans / 2, 2 * Length*trans, Length*trans, wood_two_block, angle);
						else if (material == 3) stretchTextureEx(renderer, mouse_x - Length*trans, mouse_y - Length*trans / 2, 2 * Length*trans, Length*trans, metal_two_block, angle);
						else if (material == 1) stretchTextureEx(renderer, mouse_x - Length*trans, mouse_y - Length*trans / 2, 2 * Length*trans, Length*trans, rock_two_block, angle);
					}
					else if (click_three_block == true) {
						if (material == 2) stretchTextureEx(renderer, mouse_x - 3 * Length*trans / 2, mouse_y - Length*trans / 2, 3 * Length*trans, Length*trans, wood_three_block, angle);
						else if (material == 3) stretchTextureEx(renderer, mouse_x - 3 * Length*trans / 2, mouse_y - Length*trans / 2, 3 * Length*trans, Length*trans, metal_three_block, angle);
						else if (material == 1) stretchTextureEx(renderer, mouse_x - 3 * Length*trans / 2, mouse_y - Length*trans / 2, 3 * Length*trans, Length*trans, rock_three_block, angle);
					}
					else if (click_hexagon_block == true) {
						if (material == 2) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, wood_hexagon_block, angle);
						else if (material == 3) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, metal_hexagon_block, angle);
						else if (material == 1) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, rock_hexagon_block, angle);
					}
					else if (click_polygon1_block == true) {
						if (material == 2) stretchTextureEx_revise(renderer, mouse_x - 19.0*Length*trans / 21.0, mouse_y - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, wood_polygon1_block, angle);
						else if (material == 3) stretchTextureEx_revise(renderer, mouse_x - 19.0*Length*trans / 21.0, mouse_y - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, metal_polygon1_block, angle);
						else if (material == 1) stretchTextureEx_revise(renderer, mouse_x - 19.0*Length*trans / 21.0, mouse_y - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, rock_polygon1_block, angle);
					}
					else if (click_trapezoid_block == true) {
						if (material == 2) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), wood_trapezoid_block, angle);
						else if (material == 3) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), metal_trapezoid_block, angle);
						else if (material == 1) stretchTextureEx_revise(renderer, mouse_x - Length*trans, mouse_y - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), rock_trapezoid_block, angle);
					}

					for (i = 0; i < population; i++) {
						if (object[i].shape->number == 1) {
							if (object[i].material.number == 2)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, wood_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, metal_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, rock_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, pre_wood_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, pre_metal_one_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, pre_rock_one_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 2) {
							if (object[i].material.number == 2) stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, wood_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, metal_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, rock_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, pre_wood_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, pre_metal_two_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, pre_rock_two_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 3) {
							if (object[i].material.number == 2) stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, wood_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, metal_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, rock_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, pre_wood_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, pre_metal_three_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, pre_rock_three_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 6) {
							if (object[i].material.number == 2) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), wood_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), metal_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), rock_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_wood_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_metal_trapezoid_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), pre_rock_trapezoid_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 8) {
							if (object[i].material.number == 2) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, wood_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, metal_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, rock_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_wood_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_metal_hexagon_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, pre_rock_hexagon_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 9) {
							if (object[i].material.number == 2) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, wood_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 3) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, metal_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 1) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, rock_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_wood_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_metal_polygon1_block, object[i].th * 180 / pi);
							else if (object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, pre_rock_polygon1_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 80) {
							stretchTextureEx(renderer, object[i].x[0] * trans - 49.96*Length*trans / 40.0, object[i].x[1] * trans - 33.02*Length*trans / 40.0, 101 * Length*trans / 40.0, 69.0*Length*trans / 40.0, stone, object[i].th * 180 / pi);
							if (!(object[i].v[0] == 0 && object[i].v[1] == 0)) {
								float x, y, theta, mag;
								x = object[i].v[0]; y = object[i].v[1];
								mag = sqrt(x*x + y*y);
								x = x / mag; y = y / mag;
								theta = asin(absolute(y));
								if (x >= 0 && y >= 0) theta = theta;
								else if (x >= 0 && y < 0) theta = 2 * pi - theta;
								else if (x < 0 && y >= 0) theta = pi - theta;
								else if (x < 0 && y < 0) theta = pi + theta;
								stretchTextureEx(renderer, object[i].x[0] * trans + x * 100 - 25, object[i].x[1] * trans + y * 100, 50, 21, velocity_arrow, theta * 180 / pi);
							}
						}
						else if (object[i].shape->number == 81) { //0.733511 0.759908
							stretchTextureEx_revise(renderer, object[i].x[0] * trans - 25.5*Length*trans / 40.0 - 3, object[i].x[1] * trans - 31.0*Length*trans / 40.0, 51 * Length*trans / 40.0, 62.0*Length*trans / 40.0, 0.733511*Length*trans, 0.759908*Length*trans, cat, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 97) {
							stretchTextureEx_revise(renderer, object[i].x[0] * trans - 2.946671f*Length*trans, object[i].x[1] * trans - 3.543418f*Length*trans, 250.0*Length*trans / 40.0, 279.0*Length*trans / 40.0, 117.809f*Length*trans / 40.0, 142.2465f*Length*trans / 40.0f, hill, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 98) {
							stretchTextureEx_revise(renderer, object[i].x[0] * trans - 77.5*Length*trans / 40.0, object[i].x[1] * trans - 4459.0*Length*trans / (219.0*40.0), 155.0*Length*trans / 40.0, 58.0*Length*trans / 40.0, 77.5*Length*trans / 40.0, 20.36*Length*trans / 40.0, ground_cone_small, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 99) {
							stretchTextureEx(renderer, object[i].x[0] * trans - 7 * Length*trans, object[i].x[1] * trans - 2 * Length*trans, 14 * Length*trans, 4 * Length*trans, large_soil, object[i].th * 180 / pi);
						}
					}

					for (i = 0; i < bomb_population; i++) {
						stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0);
					}
					/*				for (k = 1; k <= bomb_lenght_divide_num; k++) {
					for (j = 0; j < bomb_angle_divide; j++) {
					stretchTextureEx(renderer, (bomb[0].x[0] + k*bomb_length_divide*cos((j*2.0*pi) / bomb_angle_divide)) * trans - 2, (bomb[0].x[1] + k*bomb_length_divide*sin((j*2.0*pi) / bomb_angle_divide)) * trans - 2, 5, 5, reddot, 0);
					}
					}*/

					SDL_RenderPresent(renderer);

					total_frame_end = SDL_GetPerformanceCounter();
					total_time = (float)(total_frame_end - total_frame_start) / ((float)SDL_GetPerformanceFrequency());
					total_delay_time = (int)(((1 / fpss)*rendering_factor - total_time) * 1000);

					if (total_delay_time>0) SDL_Delay(total_delay_time);
					else SDL_Delay(1);

					if (stage_prepare != false && stage_start != false) SDL_RenderClear(renderer);
				}
				if (stage_start == true && quit == false) {

					if (explo == false) {
						explo = true;
						Mix_PlayChannel(-1, explosion, 0);
						bomb_function(object, population, bomb, bomb_population);
					}

					time_ = 0;
					float accumulator = 0;
					int framestart = SDL_GetPerformanceCounter();
					int currenttime;
					float initial_accumulate;
					float max_height = FLT_MAX;
					bool end = false;
					bool clear = false;
					bool fade_out_earthquake_sound = false;
					char text_string[50];
					int star_framestart; int star_currenttime; float star_deltime = 0; float star_accumulate = 0;
					float rendering_factor;
					if (graphic_quality == 1) rendering_factor = 1.0;
					else if (graphic_quality == 0) rendering_factor = 2.0;
					else if (graphic_quality == -1) rendering_factor = 4.0;

					float star_vx[10] = { 2.0f,1.0f,-1.0f,5.0f,3.0f,-0.5f,-4.0f,0.9f,2.4f,-3.1f };
					float star_vy[10] = { 7.0f,3.0f,2.0f,5.0f,4.0f,4.6f,2.8f,1.2f,3.5f,4.8f };
					float star_x[10] = { 454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans ,454 / trans };
					float star_y[10] = { 252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans ,252 / trans };
					float star_th[10] = { 0.0f,1.0f,2.0f,-1.0f,-2.0f,1.2f,1.3f,-2.4f,1.0f,3.7f };
					float star_w[10] = { 0.1f,0.2f,0.3f,0.4f,-0.1f,-0.5f,-0.8f,3.5f,1.2f,-5.3f };

					bool cat_hurt = false;
					float cat_hurt_coordinate[2];
					int cat_framestart; int cat_currenttime; float cat_deltime = 0; float cat_accumulate = 0;
					float cat_vx[10] = { 2.0f,1.0f,-1.0f,5.0f,3.0f,-0.5f,-4.0f,0.9f,2.4f,-3.1f };
					float cat_vy[10] = { 7.0f,3.0f,2.0f,5.0f,4.0f,4.6f,2.8f,1.2f,3.5f,4.8f };
					float cat_x[10];
					float cat_y[10];
					float cat_th[10] = { 0.0f,1.0f,2.0f,-1.0f,-2.0f,1.2f,1.3f,-2.4f,1.0f,3.7f };
					float cat_w[10] = { 0.1f,0.2f,0.3f,0.4f,-0.1f,-0.5f,-0.8f,3.5f,1.2f,-5.3f };

					int total_frame_start;
					int total_frame_end;
					float total_time;
					int total_delay_time;

					if (earthquake_type != 0) {
						Mix_PlayChannel(-1, earthquake_sound, 0);
					}

					if (speed == 0) Mix_Pause(-1);

					while (quit == false && stage_start == true) {
						total_frame_start = SDL_GetPerformanceCounter();

						currenttime = SDL_GetPerformanceCounter();
						accumulator = accumulator + (float)(currenttime - framestart) / ((float)SDL_GetPerformanceFrequency());
						framestart = SDL_GetPerformanceCounter();

						initial_accumulate = accumulator;
						if (accumulator > 0.2) accumulator = 0.2;

						float max_speed_BB = -FLT_MAX;
						float max_speed_BE = -FLT_MAX;

						while (accumulator>(1 / fpss)*rendering_factor && time_<endtime) {
							if (speed != 0) {
								if (time_ < starttime) {
									one_term(object, population, contact);
									time_ = time_ + (1 / fpss);
								}
								else if (time_ < timeduration) one_term_with_earthquake(object, population, contact, earthquake_type, &time_, amplitude, frequency);
								else if (time_ < endtime) {
									one_term(object, population, contact);
									time_ = time_ + (1 / fpss);
								}
								accumulator = accumulator - (1 / fpss) / speed;
								cat_hurt_check(object, population, contact, &cat_hurt, cat_hurt_coordinate);
								if (cat_hurt == true) break;
							}
							else {
								accumulator = accumulator - (1 / fpss);
							}
						}

						if (time_ < endtime) {
							if (max_speed_BB > 0) {
								float ratio = (max_speed_BB - sound_threshold_speed) / sound_max_speed;
								if (ratio > 0) {
									if (ratio > 1) ratio = 1;
									Mix_VolumeChunk(collide_BB, (int)(chunk_sound * ratio));
									Mix_PlayChannel(-1, collide_BB, 0);
								}
							}
							if (max_speed_BE > 0) {
								float ratio = (max_speed_BE - sound_threshold_speed) / sound_max_speed;
								if (ratio > 0) {
									if (ratio > 1) ratio = 1;
									Mix_VolumeChunk(collide_BE, (int)(chunk_sound * ratio));
									Mix_PlayChannel(-1, collide_BE, 0);
								}
							}
						}

						if (time_ > timeduration && fade_out_earthquake_sound == false) {
							Mix_FadeOutChannel(-1, 1000);
							fade_out_earthquake_sound = true;
						}

						if (time_ < endtime) {
							while (SDL_PollEvent(&event)) {
								switch (event.type) {
								case SDL_QUIT: quit = true; break;
								case SDL_MOUSEBUTTONDOWN: {
									if (event.button.button == SDL_BUTTON_LEFT) {
										int x, y;
										x = event.motion.x;
										y = event.motion.y;
										if (x > 1117 && x < 1160 && y>688 && y < 738) { speed = 0; Mix_Pause(-1); Mix_PlayChannel(-1, click_sound, 0); }
										else if (x > 1186 && x < 1234 && y>688 && y < 739) { speed = 1; Mix_Resume(-1); Mix_PlayChannel(-1, click_sound, 0); }
										else if (x > 1255 && x < 1347 && y>688 && y < 739) { speed = 2; Mix_Resume(-1); Mix_PlayChannel(-1, click_sound, 0); }
										else if (x >= 1046 && x < 1095 && y >= 685 && y < 738) { // click retry button
											stage_start = false;
											stage_prepare = true;
											SDL_DestroyTexture(map);
											SDL_DestroyTexture(right_menu);
//											SDL_DestroyTexture(arrow);
											SDL_DestroyTexture(wood_one_block);
											SDL_DestroyTexture(wood_two_block);
											SDL_DestroyTexture(wood_three_block);
											SDL_DestroyTexture(wood_hexagon_block);
											SDL_DestroyTexture(wood_polygon1_block);
											SDL_DestroyTexture(wood_trapezoid_block);
											SDL_DestroyTexture(pre_wood_one_block);
											SDL_DestroyTexture(pre_wood_two_block);
											SDL_DestroyTexture(pre_wood_three_block);
											SDL_DestroyTexture(pre_wood_hexagon_block);
											SDL_DestroyTexture(pre_wood_polygon1_block);
											SDL_DestroyTexture(pre_wood_trapezoid_block);
											SDL_DestroyTexture(metal_one_block);
											SDL_DestroyTexture(metal_two_block);
											SDL_DestroyTexture(metal_three_block);
											SDL_DestroyTexture(metal_hexagon_block);
											SDL_DestroyTexture(metal_polygon1_block);
											SDL_DestroyTexture(metal_trapezoid_block);
											SDL_DestroyTexture(pre_metal_one_block);
											SDL_DestroyTexture(pre_metal_two_block);
											SDL_DestroyTexture(pre_metal_three_block);
											SDL_DestroyTexture(pre_metal_hexagon_block);
											SDL_DestroyTexture(pre_metal_polygon1_block);
											SDL_DestroyTexture(pre_metal_trapezoid_block);
											SDL_DestroyTexture(rock_one_block);
											SDL_DestroyTexture(rock_two_block);
											SDL_DestroyTexture(rock_three_block);
											SDL_DestroyTexture(rock_hexagon_block);
											SDL_DestroyTexture(rock_polygon1_block);
											SDL_DestroyTexture(rock_trapezoid_block);
											SDL_DestroyTexture(pre_rock_one_block);
											SDL_DestroyTexture(pre_rock_two_block);
											SDL_DestroyTexture(pre_rock_three_block);
											SDL_DestroyTexture(pre_rock_hexagon_block);
											SDL_DestroyTexture(pre_rock_polygon1_block);
											SDL_DestroyTexture(pre_rock_trapezoid_block);
											SDL_DestroyTexture(explosion1);
											SDL_DestroyTexture(explosion2);
											SDL_DestroyTexture(explosion3);
											SDL_DestroyTexture(explosion4);
											SDL_DestroyTexture(explosion5);
											SDL_DestroyTexture(explosion6);
											SDL_DestroyTexture(explosion7);
											SDL_DestroyTexture(explosion8);
											SDL_DestroyTexture(explosion9);
											SDL_DestroyTexture(explosion11);
											SDL_DestroyTexture(explosion10);
											SDL_DestroyTexture(explosion12);
											SDL_DestroyTexture(explosion13);
											SDL_DestroyTexture(explosion14);
											SDL_DestroyTexture(explosion15);
											SDL_DestroyTexture(explosion16);
											SDL_DestroyTexture(explosion17);
											SDL_DestroyTexture(bombimg);
											SDL_DestroyTexture(cat);
											SDL_DestroyTexture(stone);
											SDL_DestroyTexture(large_soil);
											SDL_DestroyTexture(ground_cone_small);
											SDL_DestroyTexture(hill);
											SDL_DestroyTexture(line);
											SDL_DestroyTexture(star);
											SDL_DestroyTexture(blood_dot);
											SDL_DestroyTexture(stop_retry);
											SDL_DestroyTexture(pause);
											SDL_DestroyTexture(normal_speed);
											SDL_DestroyTexture(x2speed);
											SDL_DestroyTexture(speed_select);
											SDL_DestroyTexture(running_end_menu);
											SDL_DestroyTexture(back_to_home);
											SDL_DestroyTexture(next_arrow);
											SDL_DestroyTexture(re_arrow);
											if (population != 0) {
												for (i = 0; i < population; i++) {
													for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
													free(object[i].shape->vertex); free(object[i].shape->normal);
													free(object[i].shape->vertex_contact_index);
													free(object[i].shape->vertex_contact);
													free(object[i].shape);
												}
												free(object);
											}
											if (population != 0) population = 0;
											free(contact);
											if (bomb_population != 0) free(bomb);
											bomb_population = 0;
											retry = true;
											Mix_HaltChannel(-1);
											Mix_PlayChannel(-1, click_sound, 0);
										}
									}
									break;
								}
								}
							}
						}

						if (cat_hurt == true) time_ = endtime + 1.0f;

						if (time_ >= endtime) {
							if (end == false) {
								for (i = 0; i < population; i++) {
									for (j = 0; j < object[i].shape->vertex_num; j++) {
										if (object[i].x[1] + object[i].shape->vertex[j][1] <= max_height) max_height = object[i].x[1] + object[i].shape->vertex[j][1];
									}
								}
								if (cat_hurt == true) {
									clear = false;
									cat_framestart = SDL_GetPerformanceCounter();
									for (i = 0; i < 10; i++) {
										cat_x[i] = cat_hurt_coordinate[0];
										cat_y[i] = cat_hurt_coordinate[1];
									}
								}
								if (max_height <= height / trans && cat_hurt == false) {
									clear = true;
									/*
									if (save.save_stage == stage_number - 1 && save.save_step == step_number - 1) {
										if (save.save_step == 7) { save.save_stage = save.save_stage + 1; save.save_step = 0; }
										else save.save_step = save.save_step + 1;
									}
									if (save.record[stage_number - 1][step_number - 1][0] < 1360 - max_height) save.record[stage_number - 1][step_number - 1][0] = 1360 - max_height;
									if (save.record[stage_number - 1][step_number - 1][1] > initial_budget - budget)save.record[stage_number - 1][step_number - 1][1] = initial_budget - budget;
								*/
									Mix_PlayChannel(-1, fanfare, 0);
								}
//								savefile = fopen("save.bin", "wb");
//								fwrite(&save, sizeof(save), 1, savefile);
//								fclose(savefile);
								star_framestart = SDL_GetPerformanceCounter();
								end = true;
							}

							if (clear == true) {
								if (star_accumulate < 3.0) {
									star_currenttime = SDL_GetPerformanceCounter();
									star_deltime = (float)(star_framestart - star_currenttime) / ((float)SDL_GetPerformanceFrequency());
									star_accumulate = star_accumulate + star_deltime;
									star_framestart = SDL_GetPerformanceCounter();
									for (i = 0; i < 10; i++) {
										star_x[i] = star_x[i] + star_vx[i] * star_deltime;
										star_vy[i] = star_vy[i] + 10 * star_deltime;
										star_y[i] = star_y[i] + star_vy[i] * star_deltime;
										star_th[i] = star_th[i] + star_w[i] * star_deltime;
									}
								}
							}

							if (cat_hurt == true) {
								if (cat_accumulate < 3.0) {
									cat_currenttime = SDL_GetPerformanceCounter();
									cat_deltime = (float)(cat_framestart - cat_currenttime) / ((float)SDL_GetPerformanceFrequency());
									cat_accumulate = cat_accumulate + cat_deltime;
									cat_framestart = SDL_GetPerformanceCounter();
									for (i = 0; i < 10; i++) {
										cat_x[i] = cat_x[i] + cat_vx[i] * cat_deltime;
										cat_vy[i] = cat_vy[i] + 10 * cat_deltime;
										cat_y[i] = cat_y[i] + cat_vy[i] * cat_deltime;
										cat_th[i] = cat_th[i] + cat_w[i] * cat_deltime;
									}
								}
							}

							while (SDL_PollEvent(&event)) {
								if (event.type == SDL_QUIT) quit = true;
								if (event.type == SDL_MOUSEBUTTONDOWN) {
									if (event.button.button == SDL_BUTTON_LEFT) {
										int x, y;
										x = event.motion.x;
										y = event.motion.y;
										if (x >= 289 && x < 410 && y >= 477 && y < 543) { // click back to stage select button
											stage_start = false;
											SDL_DestroyTexture(map);
											SDL_DestroyTexture(right_menu);
											SDL_DestroyTexture(wood_one_block);
											SDL_DestroyTexture(wood_two_block);
											SDL_DestroyTexture(wood_three_block);
											SDL_DestroyTexture(wood_hexagon_block);
											SDL_DestroyTexture(wood_polygon1_block);
											SDL_DestroyTexture(wood_trapezoid_block);
											SDL_DestroyTexture(pre_wood_one_block);
											SDL_DestroyTexture(pre_wood_two_block);
											SDL_DestroyTexture(pre_wood_three_block);
											SDL_DestroyTexture(pre_wood_hexagon_block);
											SDL_DestroyTexture(pre_wood_polygon1_block);
											SDL_DestroyTexture(pre_wood_trapezoid_block);
											SDL_DestroyTexture(metal_one_block);
											SDL_DestroyTexture(metal_two_block);
											SDL_DestroyTexture(metal_three_block);
											SDL_DestroyTexture(metal_hexagon_block);
											SDL_DestroyTexture(metal_polygon1_block);
											SDL_DestroyTexture(metal_trapezoid_block);
											SDL_DestroyTexture(pre_metal_one_block);
											SDL_DestroyTexture(pre_metal_two_block);
											SDL_DestroyTexture(pre_metal_three_block);
											SDL_DestroyTexture(pre_metal_hexagon_block);
											SDL_DestroyTexture(pre_metal_polygon1_block);
											SDL_DestroyTexture(pre_metal_trapezoid_block);
											SDL_DestroyTexture(rock_one_block);
											SDL_DestroyTexture(rock_two_block);
											SDL_DestroyTexture(rock_three_block);
											SDL_DestroyTexture(rock_hexagon_block);
											SDL_DestroyTexture(rock_polygon1_block);
											SDL_DestroyTexture(rock_trapezoid_block);
											SDL_DestroyTexture(pre_rock_one_block);
											SDL_DestroyTexture(pre_rock_two_block);
											SDL_DestroyTexture(pre_rock_three_block);
											SDL_DestroyTexture(pre_rock_hexagon_block);
											SDL_DestroyTexture(pre_rock_polygon1_block);
											SDL_DestroyTexture(pre_rock_trapezoid_block);
											SDL_DestroyTexture(explosion1);
											SDL_DestroyTexture(explosion2);
											SDL_DestroyTexture(explosion3);
											SDL_DestroyTexture(explosion4);
											SDL_DestroyTexture(explosion5);
											SDL_DestroyTexture(explosion6);
											SDL_DestroyTexture(explosion7);
											SDL_DestroyTexture(explosion8);
											SDL_DestroyTexture(explosion9);
											SDL_DestroyTexture(explosion11);
											SDL_DestroyTexture(explosion10);
											SDL_DestroyTexture(explosion12);
											SDL_DestroyTexture(explosion13);
											SDL_DestroyTexture(explosion14);
											SDL_DestroyTexture(explosion15);
											SDL_DestroyTexture(explosion16);
											SDL_DestroyTexture(explosion17);
											SDL_DestroyTexture(bombimg);
											SDL_DestroyTexture(cat);
											SDL_DestroyTexture(stone);
											SDL_DestroyTexture(large_soil);
											SDL_DestroyTexture(ground_cone_small);
											SDL_DestroyTexture(hill);
											SDL_DestroyTexture(line);
											SDL_DestroyTexture(star);
											SDL_DestroyTexture(blood_dot);
											SDL_DestroyTexture(stop_retry);
											SDL_DestroyTexture(pause);
											SDL_DestroyTexture(normal_speed);
											SDL_DestroyTexture(x2speed);
											SDL_DestroyTexture(speed_select);
											SDL_DestroyTexture(running_end_menu);
											SDL_DestroyTexture(back_to_home);
											SDL_DestroyTexture(next_arrow);
											SDL_DestroyTexture(re_arrow);
											map_running = false;
											if (population != 0) {
												for (i = 0; i < population; i++) {
													for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
													free(object[i].shape->vertex); free(object[i].shape->normal);
													free(object[i].shape->vertex_contact_index);
													free(object[i].shape->vertex_contact);
													free(object[i].shape);
												}
												free(object);
											}
											free(contact);
											if (population != 0) {
												for (i = 0; i < population; i++) {
													for (j = 0; j < pre_object[i].shape->vertex_num; j++) { free(pre_object[i].shape->vertex[j]); free(pre_object[i].shape->normal[j]); }
													free(pre_object[i].shape->vertex); free(pre_object[i].shape->normal);
													free(pre_object[i].shape->vertex_contact_index);
													free(pre_object[i].shape->vertex_contact);
													free(pre_object[i].shape);
												}
												free(pre_object);
											}
											if (population != 0) population = 0;
											if (bomb_population != 0) free(bomb);
											bomb_population = 0;
											retry = false;
											stage_to_select_menu = true;
											Mix_PlayChannel(-1, click_sound, 0);
											Mix_HaltMusic();
											Mix_FadeInMusic(main_music, -1, 1000);
										}

										else if (x >= 513 && x < 580 && y >= 472 && y < 546) { // click retry button
											stage_start = false;
											stage_prepare = true;
											SDL_DestroyTexture(map);
											SDL_DestroyTexture(right_menu);
//											SDL_DestroyTexture(arrow);
											SDL_DestroyTexture(wood_one_block);
											SDL_DestroyTexture(wood_two_block);
											SDL_DestroyTexture(wood_three_block);
											SDL_DestroyTexture(wood_hexagon_block);
											SDL_DestroyTexture(wood_polygon1_block);
											SDL_DestroyTexture(wood_trapezoid_block);
											SDL_DestroyTexture(pre_wood_one_block);
											SDL_DestroyTexture(pre_wood_two_block);
											SDL_DestroyTexture(pre_wood_three_block);
											SDL_DestroyTexture(pre_wood_hexagon_block);
											SDL_DestroyTexture(pre_wood_polygon1_block);
											SDL_DestroyTexture(pre_wood_trapezoid_block);
											SDL_DestroyTexture(metal_one_block);
											SDL_DestroyTexture(metal_two_block);
											SDL_DestroyTexture(metal_three_block);
											SDL_DestroyTexture(metal_hexagon_block);
											SDL_DestroyTexture(metal_polygon1_block);
											SDL_DestroyTexture(metal_trapezoid_block);
											SDL_DestroyTexture(pre_metal_one_block);
											SDL_DestroyTexture(pre_metal_two_block);
											SDL_DestroyTexture(pre_metal_three_block);
											SDL_DestroyTexture(pre_metal_hexagon_block);
											SDL_DestroyTexture(pre_metal_polygon1_block);
											SDL_DestroyTexture(pre_metal_trapezoid_block);
											SDL_DestroyTexture(rock_one_block);
											SDL_DestroyTexture(rock_two_block);
											SDL_DestroyTexture(rock_three_block);
											SDL_DestroyTexture(rock_hexagon_block);
											SDL_DestroyTexture(rock_polygon1_block);
											SDL_DestroyTexture(rock_trapezoid_block);
											SDL_DestroyTexture(pre_rock_one_block);
											SDL_DestroyTexture(pre_rock_two_block);
											SDL_DestroyTexture(pre_rock_three_block);
											SDL_DestroyTexture(pre_rock_hexagon_block);
											SDL_DestroyTexture(pre_rock_polygon1_block);
											SDL_DestroyTexture(pre_rock_trapezoid_block);
											SDL_DestroyTexture(explosion1);
											SDL_DestroyTexture(explosion2);
											SDL_DestroyTexture(explosion3);
											SDL_DestroyTexture(explosion4);
											SDL_DestroyTexture(explosion5);
											SDL_DestroyTexture(explosion6);
											SDL_DestroyTexture(explosion7);
											SDL_DestroyTexture(explosion8);
											SDL_DestroyTexture(explosion9);
											SDL_DestroyTexture(explosion11);
											SDL_DestroyTexture(explosion10);
											SDL_DestroyTexture(explosion12);
											SDL_DestroyTexture(explosion13);
											SDL_DestroyTexture(explosion14);
											SDL_DestroyTexture(explosion15);
											SDL_DestroyTexture(explosion16);
											SDL_DestroyTexture(explosion17);
											SDL_DestroyTexture(bombimg);
											SDL_DestroyTexture(cat);
											SDL_DestroyTexture(stone);
											SDL_DestroyTexture(large_soil);
											SDL_DestroyTexture(ground_cone_small);
											SDL_DestroyTexture(hill);
											SDL_DestroyTexture(line);
											SDL_DestroyTexture(star);
											SDL_DestroyTexture(blood_dot);
											SDL_DestroyTexture(stop_retry);
											SDL_DestroyTexture(pause);
											SDL_DestroyTexture(normal_speed);
											SDL_DestroyTexture(x2speed);
											SDL_DestroyTexture(speed_select);
											SDL_DestroyTexture(running_end_menu);
											SDL_DestroyTexture(back_to_home);
											SDL_DestroyTexture(next_arrow);
											SDL_DestroyTexture(re_arrow);
											if (population != 0) {
												for (i = 0; i < population; i++) {
													for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
													free(object[i].shape->vertex); free(object[i].shape->normal);
													free(object[i].shape->vertex_contact_index);
													free(object[i].shape->vertex_contact);
													free(object[i].shape);
												}
												free(object);
											}
											if (population != 0) population = 0;
											free(contact);
											if (bomb_population != 0) free(bomb);
											bomb_population = 0;
											retry = true;
											Mix_PlayChannel(-1, click_sound, 0);
										}
										/*else if (x >= 688 && x<779 && y >= 477 && y<841 && clear == true && !(stage_number == 5 && step_number == 8)) { // click next button
											stage_start = false;
											stage_prepare = true;
											SDL_DestroyTexture(map);
											SDL_DestroyTexture(right_menu);
											SDL_DestroyTexture(arrow);
											SDL_DestroyTexture(wood_one_block);
											SDL_DestroyTexture(wood_two_block);
											SDL_DestroyTexture(wood_three_block);
											SDL_DestroyTexture(wood_hexagon_block);
											SDL_DestroyTexture(wood_polygon1_block);
											SDL_DestroyTexture(wood_trapezoid_block);
											SDL_DestroyTexture(pre_wood_one_block);
											SDL_DestroyTexture(pre_wood_two_block);
											SDL_DestroyTexture(pre_wood_three_block);
											SDL_DestroyTexture(pre_wood_hexagon_block);
											SDL_DestroyTexture(pre_wood_polygon1_block);
											SDL_DestroyTexture(pre_wood_trapezoid_block);
											SDL_DestroyTexture(metal_one_block);
											SDL_DestroyTexture(metal_two_block);
											SDL_DestroyTexture(metal_three_block);
											SDL_DestroyTexture(metal_hexagon_block);
											SDL_DestroyTexture(metal_polygon1_block);
											SDL_DestroyTexture(metal_trapezoid_block);
											SDL_DestroyTexture(pre_metal_one_block);
											SDL_DestroyTexture(pre_metal_two_block);
											SDL_DestroyTexture(pre_metal_three_block);
											SDL_DestroyTexture(pre_metal_hexagon_block);
											SDL_DestroyTexture(pre_metal_polygon1_block);
											SDL_DestroyTexture(pre_metal_trapezoid_block);
											SDL_DestroyTexture(rock_one_block);
											SDL_DestroyTexture(rock_two_block);
											SDL_DestroyTexture(rock_three_block);
											SDL_DestroyTexture(rock_hexagon_block);
											SDL_DestroyTexture(rock_polygon1_block);
											SDL_DestroyTexture(rock_trapezoid_block);
											SDL_DestroyTexture(pre_rock_one_block);
											SDL_DestroyTexture(pre_rock_two_block);
											SDL_DestroyTexture(pre_rock_three_block);
											SDL_DestroyTexture(pre_rock_hexagon_block);
											SDL_DestroyTexture(pre_rock_polygon1_block);
											SDL_DestroyTexture(pre_rock_trapezoid_block);
											SDL_DestroyTexture(explosion1);
											SDL_DestroyTexture(explosion2);
											SDL_DestroyTexture(explosion3);
											SDL_DestroyTexture(explosion4);
											SDL_DestroyTexture(explosion5);
											SDL_DestroyTexture(explosion6);
											SDL_DestroyTexture(explosion7);
											SDL_DestroyTexture(explosion8);
											SDL_DestroyTexture(explosion9);
											SDL_DestroyTexture(explosion11);
											SDL_DestroyTexture(explosion10);
											SDL_DestroyTexture(explosion12);
											SDL_DestroyTexture(explosion13);
											SDL_DestroyTexture(explosion14);
											SDL_DestroyTexture(explosion15);
											SDL_DestroyTexture(explosion16);
											SDL_DestroyTexture(explosion17);
											SDL_DestroyTexture(bombimg);
											SDL_DestroyTexture(cat);
											SDL_DestroyTexture(stone);
											SDL_DestroyTexture(large_soil);
											SDL_DestroyTexture(ground_cone_small);
											SDL_DestroyTexture(hill);
											SDL_DestroyTexture(line);
											SDL_DestroyTexture(star);
											SDL_DestroyTexture(blood_dot);
											SDL_DestroyTexture(stop_retry);
											SDL_DestroyTexture(pause);
											SDL_DestroyTexture(normal_speed);
											SDL_DestroyTexture(x2speed);
											SDL_DestroyTexture(speed_select);
											SDL_DestroyTexture(running_end_menu);
											SDL_DestroyTexture(back_to_home);
											SDL_DestroyTexture(next_arrow);
											SDL_DestroyTexture(re_arrow);
											if (population != 0) {
												for (i = 0; i < population; i++) {
													for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
													free(object[i].shape->vertex); free(object[i].shape->normal);
													free(object[i].shape->vertex_contact_index);
													free(object[i].shape->vertex_contact);
													free(object[i].shape);
												}
												free(object);
											}
											if (population != 0) population = 0;
											free(contact);
											if (bomb_population != 0) free(bomb);
											bomb_population = 0;
											if (step_number == 8) {
												stage_number = stage_number + 1;
												step_number = 1;
											}
											else step_number = step_number + 1;
											next_click = true;
										}*/
										else if (x > 1117 && x < 1160 && y>688 && y < 738) speed = 0;
										else if (x > 1186 && x < 1234 && y>688 && y < 739) speed = 1;
										else if (x > 1255 && x < 1347 && y>688 && y < 739) speed = 2;
									}
								}
							}

						}

						drawTexture(renderer, 0, 0, map);
						drawTexture(renderer, 0, height, line);
						drawTexture(renderer, 1004, 0, right_menu);
						if (speed == 0) drawTexture(renderer, 1090, 664, speed_select);
						else if (speed == 1)drawTexture(renderer, 1162, 664, speed_select);
						else if (speed == 2) drawTexture(renderer, 1253, 664, speed_select);
						drawTexture(renderer, 1046, 685, stop_retry);
						drawTexture(renderer, 1117, 688, pause);
						drawTexture(renderer, 1186, 688, normal_speed);
						drawTexture(renderer, 1255, 688, x2speed);

						for (i = 0; i < population; i++) {
							switch (object[i].shape->number) {
							case 1:
								if (object[i].material.number == 2 || object[i].material.number == 5)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, wood_one_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 3 || object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, metal_one_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 1 || object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, Length*trans, Length*trans, rock_one_block, object[i].th * 180 / pi);
								break;
							case 2:
								if (object[i].material.number == 2 || object[i].material.number == 5) stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, wood_two_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 3 || object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, metal_two_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 1 || object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - Length*trans / 2, 2 * Length*trans, Length*trans, rock_two_block, object[i].th * 180 / pi);
								break;
							case 3:
								if (object[i].material.number == 2 || object[i].material.number == 5) stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, wood_three_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 3 || object[i].material.number == 6)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, metal_three_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 1 || object[i].material.number == 4)stretchTextureEx(renderer, object[i].x[0] * trans - 3 * Length*trans / 2, object[i].x[1] * trans - Length*trans / 2, 3 * Length*trans, Length*trans, rock_three_block, object[i].th * 180 / pi);
								break;
							case 6:
								if (object[i].material.number == 2 || object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), wood_trapezoid_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 3 || object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), metal_trapezoid_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 1 || object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 4.0 - 2, 2 * Length*trans, sqrt(3)*Length*trans / 2.0 + 1, Length*trans, Length*trans*5.0 / (6.0*sqrt(3)), rock_trapezoid_block, object[i].th * 180 / pi);
								break;
							case 8:
								if (object[i].material.number == 2 || object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, wood_hexagon_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 3 || object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, metal_hexagon_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 1 || object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - Length*trans, object[i].x[1] * trans - sqrt(3)*Length*trans / 2.0, 2 * Length*trans, sqrt(3)*Length*trans + 1, Length*trans, sqrt(3)*Length*trans / 2.0, rock_hexagon_block, object[i].th * 180 / pi);
								break;
							case 9:
								if (object[i].material.number == 2 || object[i].material.number == 5) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, wood_polygon1_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 3 || object[i].material.number == 6) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, metal_polygon1_block, object[i].th * 180 / pi);
								else if (object[i].material.number == 1 || object[i].material.number == 4) stretchTextureEx_revise(renderer, object[i].x[0] * trans - 19.0*Length*trans / 21.0, object[i].x[1] * trans - 23.0*Length*trans / 21.0, 2 * Length*trans, 2 * Length*trans, 19.0*Length*trans / 21.0, 23.0*Length*trans / 21.0, rock_polygon1_block, object[i].th * 180 / pi);
								break;
							case 80:
								stretchTextureEx(renderer, object[i].x[0] * trans - 49.96*Length*trans / 40.0, object[i].x[1] * trans - 33.02*Length*trans / 40.0, 101 * Length*trans / 40.0, 69.0*Length*trans / 40.0, stone, object[i].th * 180 / pi);
								break;
							case 81:
								stretchTextureEx_revise(renderer, object[i].x[0] * trans - 25.5*Length*trans / 40.0 - 3, object[i].x[1] * trans - 31.0*Length*trans / 40.0, 51 * Length*trans / 40.0, 62.0*Length*trans / 40.0, 0.733511*Length*trans, 0.759908*Length*trans, cat, object[i].th * 180 / pi);
								break;
							case 97:
								stretchTextureEx_revise(renderer, object[i].x[0] * trans - 2.946671f*Length*trans, object[i].x[1] * trans - 3.543418f*Length*trans, 250.0*Length*trans / 40.0, 279.0*Length*trans / 40.0, 117.809f*Length*trans / 40.0, 142.2465f*Length*trans / 40.0f, hill, object[i].th * 180 / pi);
								break;
							case 98:
								stretchTextureEx_revise(renderer, object[i].x[0] * trans - 77.5*Length*trans / 40.0, object[i].x[1] * trans - 4459.0*Length*trans / (219.0*40.0), 155.0*Length*trans / 40.0, 58.0*Length*trans / 40.0, 77.5*Length*trans / 40.0, 20.36*Length*trans / 40.0, ground_cone_small, object[i].th * 180 / pi);
								break;
							case 99:
								stretchTextureEx(renderer, object[i].x[0] * trans - 7 * Length*trans, object[i].x[1] * trans - 2 * Length*trans, 14 * Length*trans, 4 * Length*trans, large_soil, object[i].th * 180 / pi);
								break;
							}
						}

	/*					for (i = 0; i < population; i++) for (j = 0; j < object[i].shape->vertex_num; j++) {
						float vertex[2];
						vertex[0] = object[i].x[0] + object[i].shape->vertex[j][0];
						vertex[1] = object[i].x[1] + object[i].shape->vertex[j][1];
						stretchTextureEx(renderer, vertex[0]*trans - 2, vertex[1]*trans - 2, 5, 5, reddot, 0);
						}*/

						if (time_ >= endtime) {

							drawTexture(renderer, 230, 207, running_end_menu);
							drawTexture(renderer, 289, 477, back_to_home);
							drawTexture(renderer, 513, 472, re_arrow);

							if (clear == true) {
								sprintf(text_string, "Clear");
								for (i = 0; i < 10; i++) stretchTextureEx(renderer, star_x[i] * trans, star_y[i] * trans, 37, 32, star, star_th[i] * 180 / pi);
							}
							else sprintf(text_string, "  Fail");
							text_surface = TTF_RenderText_Blended(font70, text_string, white);
							text = SDL_CreateTextureFromSurface(renderer, text_surface);
							drawTexture(renderer, 454, 222, text);
							drawTexture(renderer, 454, 222, text);
							drawTexture(renderer, 454, 222, text);
							SDL_FreeSurface(text_surface);
							SDL_DestroyTexture(text);

							if (cat_hurt == true) for (i = 0; i < 10; i++) stretchTextureEx(renderer, cat_x[i] * trans, cat_y[i] * trans, 10, 10, blood_dot, cat_th[i] * 180 / pi);

							if (1360 - max_height >= 0) sprintf(text_string, "final height  %.1fm", (1360 - max_height)*0.01);
							else sprintf(text_string, "final height  x");
							text_surface = TTF_RenderText_Blended(font32, text_string, white);
							text = SDL_CreateTextureFromSurface(renderer, text_surface);
							drawTexture(renderer, 416, 298, text);
							drawTexture(renderer, 416, 298, text);
							drawTexture(renderer, 416, 298, text);
							SDL_FreeSurface(text_surface);
							SDL_DestroyTexture(text);

							sprintf(text_string, "used budget  %d$", initial_budget - budget);
							text_surface = TTF_RenderText_Blended(font32, text_string, white);
							text = SDL_CreateTextureFromSurface(renderer, text_surface);
							drawTexture(renderer, 414, 341, text);
							drawTexture(renderer, 414, 341, text);
							drawTexture(renderer, 414, 341, text);
							SDL_FreeSurface(text_surface);
							SDL_DestroyTexture(text);
							/*
							if (save.record[stage_number - 1][step_number - 1][0] >= 0) sprintf(text_string, "your highest final height  %.1fm", save.record[stage_number - 1][step_number - 1][0] * 0.01);
							else sprintf(text_string, "your highest final height  x");
							text_surface = TTF_RenderText_Blended(font32, text_string, white);
							text = SDL_CreateTextureFromSurface(renderer, text_surface);
							drawTexture(renderer, 311, 381, text);
							drawTexture(renderer, 311, 381, text);
							drawTexture(renderer, 311, 381, text);
							SDL_FreeSurface(text_surface);
							SDL_DestroyTexture(text);

							if (save.record[stage_number - 1][step_number - 1][1] != FLT_MAX) sprintf(text_string, "your lowest used budget  %.0f$", save.record[stage_number - 1][step_number - 1][1]);
							else sprintf(text_string, "your lowest used budget  x");
							text_surface = TTF_RenderText_Blended(font32, text_string, white);
							text = SDL_CreateTextureFromSurface(renderer, text_surface);
							drawTexture(renderer, 323, 421, text);
							drawTexture(renderer, 323, 421, text);
							drawTexture(renderer, 323, 421, text);
							SDL_FreeSurface(text_surface);
							SDL_DestroyTexture(text);
							*/
						}

						if (time_<bomb_animation_time * 17) {
							for (i = 0; i < bomb_population; i++) {
								if (time_ < bomb_animation_time) { stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0); stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion1, 0); }
								else if (time_<bomb_animation_time * 2) { stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0); stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion2, 0); }
								else if (time_ < bomb_animation_time * 3) { stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0); stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion3, 0); }
								else if (time_ < bomb_animation_time * 4) { stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0); stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion4, 0); }
								else if (time_ < bomb_animation_time * 5) { stretchTextureEx(renderer, bomb[i].x[0] * trans - 35, bomb[i].x[1] * trans - 45, 70, 70, bombimg, 0); stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion5, 0); }
								else if (time_<bomb_animation_time * 6) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion6, 0);
								else if (time_<bomb_animation_time * 7) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion7, 0);
								else if (time_<bomb_animation_time * 8) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion8, 0);
								else if (time_<bomb_animation_time * 9) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion9, 0);
								else if (time_<bomb_animation_time * 10) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion10, 0);
								else if (time_<bomb_animation_time * 11) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion11, 0);
								else if (time_<bomb_animation_time * 12) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion12, 0);
								else if (time_<bomb_animation_time * 13) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion13, 0);
								else if (time_<bomb_animation_time * 14) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion14, 0);
								else if (time_<bomb_animation_time * 15) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion15, 0);
								else if (time_<bomb_animation_time * 16) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion16, 0);
								else if (time_<bomb_animation_time * 17) stretchTextureEx(renderer, bomb[i].x[0] * trans - 150, bomb[i].x[1] * trans - 150, 300, 300, explosion17, 0);
							}
						}

						if (retry == false && stage_to_select_menu == false && next_click == false) SDL_RenderPresent(renderer);

						total_frame_end = SDL_GetPerformanceCounter();
						total_time = (float)(total_frame_end - total_frame_start) / ((float)SDL_GetPerformanceFrequency());
						total_delay_time = (int)(((1 / fpss)*rendering_factor - total_time) * 1000);

						if (total_delay_time>0) SDL_Delay(total_delay_time);
						else SDL_Delay(1);
						SDL_RenderClear(renderer);
					}
				}
			}
		}
		if (retry == false && stage_to_select_menu==false && next_click == false && escape_map_editor == false) SDL_RenderPresent(renderer);
		total_frame_end = SDL_GetPerformanceCounter();
		total_time = (float)(total_frame_end - total_frame_start) / ((float)SDL_GetPerformanceFrequency());
		total_delay_time = (int)(((1 / fpss)*rendering_factor - total_time) * 1000);

		if (total_delay_time>0) SDL_Delay(total_delay_time);
		else SDL_Delay(1);
		SDL_RenderClear(renderer);
	}







/*	while (!quit) {

		for (i = 0; i < number; i++) one_term(A, number);
		draw(A, number);

		if (SDL_PollEvent(&event)) {
			switch (event.type) {
			case SDL_QUIT:
					quit = true;
					break;
			}
		}
		printf("%f\n", A[1].th*180/pi);
		SDL_RenderClear(renderer);
		SDL_Delay(10);
	}

	SDL_Quit();
	*/
	TTF_Quit();
	Mix_Quit();
	return 0;

}

void draw(Object A[], int NumberOfObject) {

	stretchTextureEx(renderer, 0, 0, 500,500, texture[0], 0);

	int i;
	for (i = 0; i < NumberOfObject; i++) {
		if (A[i].shape->number == 1) {
			stretchTextureEx(renderer, A[i].x[0] - Length / 2, A[i].x[1] - Length / 2, Length, Length, texture[1], A[i].th * 180 / pi);
		}
		else if (A[i].shape->number == 2) {
			stretchTextureEx(renderer, A[i].x[0] - 3 * Length / 2, A[i].x[1] - Length / 2, 3 * Length,Length, texture[1], A[i].th * 180 / pi);
		}
		else if (A[i].shape->number == 99) {
			stretchTextureEx(renderer, A[i].x[0] - 9 * Length, A[i].x[1] - Length, 18 * Length, 2*Length, texture[2], A[i].th * 180 / pi);
		}
	}
		SDL_RenderPresent(renderer);
//		SDL_DestroyTexture(texture);
}

void cycle1(Object A[], int n, int NumberOfObject, cinf contact[]) { // A[n]
	int i;

	for (i = 0; i < NumberOfObject; i++) if (i > n) {
		find_impulse(&A[n], &A[i], NumberOfObject, contact, n, i);
		find_impulse(&A[i], &A[n], NumberOfObject, contact, i, n);
	}
}

void one_term(Object A[], int NumberOfObject, cinf contact[]) {
	int i, j, k;

	for (i = 0; i < NumberOfObject; i++) {
		A[i].delv[0] = 0;
		A[i].delv[1] = 0;
		A[i].delw = 0;
	}

	for (i = 0; i < NumberOfObject; i++) { force_zero(&A[i]); gravity(&A[i]); }
	for (i = 0; i < NumberOfObject; i++) { apply_force(&A[i]); }

	for (i = 0; i < NumberOfObject; i++) {
		for (j = 0; j < NumberOfObject; j++) {
			if (i > j ) if(!(A[i].shape->layer>0 && A[j].shape->layer>0 && (A[i].shape->layer== A[j].shape->layer))) { collision_check(&A[j], &A[i], NumberOfObject, contact, j, i); collision_check(&A[i], &A[j], NumberOfObject, contact, i, j); }
		}
	}

	uncollide_check(A, NumberOfObject, contact);
	warmstarting(A, NumberOfObject, contact);

	for (j = 0; j < 10; j++) { 
		for (i = 0; i < NumberOfObject; i++) {
			cycle1(A, i, NumberOfObject, contact);
		}
	}

	for (i = 0; i < NumberOfObject; i++) { movement(&A[i]); }
	for (k = 0; k < 3;k++) for (i = 0; i < NumberOfObject; i++) for (j = 0; j < NumberOfObject; j++) if (i > j) if (!(A[i].shape->layer>0 && A[j].shape->layer>0 && (A[i].shape->layer == A[j].shape->layer))) { positionalcorrection(&A[i], &A[j]); positionalcorrection(&A[j], &A[i]); }
}

void set_object(Object * A, int shape, int material, float x1, float x2, float v1, float v2, float a1, float a2, float th, float w, float alp, float F1, float F2) {
	A->shape = (Shape *)malloc(sizeof(Shape));
	set_shape(A->shape, shape);
	set_material(A, material);
	set_mass(A);
	A->x[0] = x1/trans; A->x[1] = x2 / trans;
	A->v[0] = v1 / trans; A->v[1] = v2 / trans;
	A->a[0] = a1 / trans; A->a[1] = a2 / trans;
	A->th = th; A->w = w; A->alp = alp;
	A->F[0] = F1; A->F[1] = F2;
	if (material == 99) A->stable = true;
	else A->stable = false;
}

void set_bomb(Bomb * b, float x1, float x2, float momentum) {
	int i;
	for (i = 0; i < bomb_angle_divide; i++) b->explosion[i] = false;
	b->momentum = momentum / (bomb_angle_divide);
	b->x[0] = x1 / trans;
	b->x[1] = x2 / trans;
}

void set_material(Object * object, int n) {
	enum stuff { rock = 1, wood, metal, pre_rock, pre_wood, pre_metal, cat = 97, stone = 98, ground = 99 };

	if (n == rock) {
		object->material.restitution = 0.1/5.0;
		object->material.density = 2.6;
		object->material.us = 0.8;
		object->material.uk = 0.6;
		object->material.number = 1;
	}
	else if (n == wood) {
		object->material.restitution = 0.2 / 5.0;
		object->material.density = 0.6;
		object->material.us = 0.4;
		object->material.uk = 0.2;
		object->material.number = 2;
	}
	else if (n == metal) {
		object->material.restitution = 0.05 / 5.0;
		object->material.density = 7.874;
		object->material.us = 0.74;
		object->material.uk = 0.57;
		object->material.number = 3;
	}
	else if (n == pre_rock) {
		object->material.restitution = 0.1 / 5.0;
		object->material.density = 2.6;
		object->material.us = 0.8;
		object->material.uk = 0.6;
		object->material.number = 4;
	}
	else if (n == pre_wood) {
		object->material.restitution = 0.2 / 5.0;
		object->material.density = 0.6;
		object->material.us = 0.4;
		object->material.uk = 0.2;
		object->material.number = 5;
	}
	else if (n == pre_metal) {
		object->material.restitution = 0.05 / 5.0;
		object->material.density = 7.874;
		object->material.us = 0.74;
		object->material.uk = 0.57;
		object->material.number = 6;
	}
	else if (n == cat) {
		object->material.restitution = 0.0f;
		object->material.density = 0.985f;
		object->material.us = 1.2f;
		object->material.uk = 1.1f;
		object->material.number = 97;
	}
	else if (n == stone) {
		object->material.restitution = 0.1 / 5.0;
		object->material.density = 2.6;
		object->material.us = 0.8;
		object->material.uk = 0.6;
		object->material.number = 98;
	}
	else if (n == ground) {
		object->material.restitution = 0.1 / 5.0;
		object->material.density = -1;
		object->material.us = 0.8;
		object->material.uk = 0.6;
		object->material.number = 99;
	}

}

void set_mass(Object * object) {
	float mass = object->shape->volume*object->material.density;
	object->inv_mass = 1 / mass;
	if (object->material.number == 99) object->inv_mass = 0;
	float inertia = object->shape->inertia_constant*mass*object->shape->volume;
	object->inv_inertia = 1 / inertia;
	if (object->material.number == 99) object->inv_inertia = 0;
}

float find_penetration_depth(Object * A, Object * B, int *indexB, int * indexA) { // find the pedetration depth of B about A: B sink to the A

	if ((A->shape->max_distance + B->shape->max_distance)*(A->shape->max_distance + B->shape->max_distance) < (A->x[0] - B->x[0])*(A->x[0] - B->x[0]) + (A->x[1] - B->x[1])*(A->x[1] - B->x[1])) return 1;

	int i,j;
	int index;
//	float * depth_array; 
	float depth_array[MAX_VERTEX];
//	depth_array = (float *)malloc((A->shape->vertex_num)*sizeof(float));
	float best_depth=-FLT_MAX;
	float a_vertex[2]; // vertex coordinate o f A about origin
	float b_vertex[2]; // vertex coordinate of B about origin

	for (i = 0; i < A->shape->vertex_num; i++) {
		b_vertex[0] = B->x[0] + B->shape->vertex[*indexB][0];
		b_vertex[1] = B->x[1] + B->shape->vertex[*indexB][1];
		a_vertex[0] = A->x[0] + A->shape->vertex[i][0];
		a_vertex[1] = A->x[1] + A->shape->vertex[i][1];
		depth_array[i] = v_dot(b_vertex, A->shape->normal[i]) - v_dot(a_vertex, A->shape->normal[i]);
		}

//	bool * select;
	bool select[MAX_VERTEX];
//	select = (bool*)malloc(sizeof(bool)*A->shape->vertex_num);
	for (i = 0; i < A->shape->vertex_num; i++) select[i] = true;

	for (j = 0; j < A->shape->vertex_num; j++) {
		for (i = 0; i < A->shape->vertex_num; i++) if (depth_array[i] > best_depth && select[i] == true) { index = i; best_depth = depth_array[i]; }
		select[index] = false;
		if (best_depth > 0) { 
//			free(depth_array); 
//			free(select); 
			return best_depth; 
		}
		if (-v_dot(B->shape->vertex[*indexB], A->shape->normal[index]) > 0) { 
			*indexA = index; 
//			free(depth_array); 
//			free(select); 
			return best_depth; 
		}
		best_depth = -FLT_MAX;
	}

//	free(depth_array);
//	free(select);
	return best_depth; // positive means there is no collision, negative means there is collision and dept is its absolute value
}

void find_impulse(Object * A, Object * B, int NumberOfObject, cinf contact[], int numA,  int numB) { // Let B sink to the A
	float e = minimum(A->material.restitution, B->material.restitution);
	float dPn; // Impulse magnitude
	float tempt_vector[2]; // temporary vector for calculus
	float VA[2]; // velocity of collision point about A
	float rA[2]; // position vector of collision point from COM of A
	float VB[2]; // velocity of collision point about B
	float rB[2]; // position vector of collision point from COM of B
	float VBA[2]; // VB-VA
	float u;
	int indexA, indexB;
	int index;
	int contact_num;
	int i;

	for (indexB = 0; indexB < B->shape->vertex_num; indexB++) {
		if (B->shape->vertex_contact_index[indexB]!=-1 && contact[B->shape->vertex_contact_index[indexB]].numA==numA) {
			index = B->shape->vertex_contact_index[indexB];
			indexA = contact[index].indexA;

			rB[0] = B->shape->vertex[indexB][0]; rB[1] = B->shape->vertex[indexB][1]; // set rB
			v_sum(B->x, rB, tempt_vector); v_sub(tempt_vector, A->x, rA); // set rA
			v_cross_k_inv(A->w, rA, tempt_vector); v_sum(tempt_vector, A->v, VA); // set VA
			VA[0] = VA[0];	VA[1] = VA[1];
			v_cross_k_inv(B->w, rB, tempt_vector); v_sum(tempt_vector, B->v, VB); // set VB
			VB[0] = VB[0];	VB[1] = VB[1];
			v_sub(VB, VA, VBA); // set VBA
			dPn = -v_dot(A->shape->normal[indexA], VBA)+contact[index].velocity_bias;
			dPn = dPn / contact[index].normal_mass; // set impulse

				float Pn0 = contact[index].normal_impulse;
				contact[index].normal_impulse = maximum(Pn0 + dPn, 0.0f);
				dPn = contact[index].normal_impulse - Pn0;

				float Pn[2]; Pn[0] = dPn*A->shape->normal[indexA][0]; Pn[1] = dPn*A->shape->normal[indexA][1];
				
				A->v[0] = A->v[0] - A->inv_mass*Pn[0]; A->v[1] = A->v[1] - A->inv_mass*Pn[1]; // impulse on A's v
				A->w = A->w - A->inv_inertia*v_cross(rA, Pn); // impulse on A's w

				B->v[0] = B->v[0] + B->inv_mass*Pn[0]; B->v[1] = B->v[1] + B->inv_mass*Pn[1]; // impulse on B's v
				B->w = B->w + B->inv_inertia*v_cross(rB, Pn); // impulse on B's w

				v_cross_k_inv(A->w, rA, tempt_vector); v_sum(tempt_vector, A->v, VA); VA[0] = VA[0]; VA[1] = VA[1]; // set VA
				v_cross_k_inv(B->w, rB, tempt_vector); v_sum(tempt_vector, B->v, VB); VB[0] = VB[0]; VB[1] = VB[1]; // set VB
				v_sub(VB, VA, VBA); // set VBA

				float tangent[2];
				v_cross_k(A->shape->normal[indexA], 1.0f, tangent);
				float vt = v_dot(VBA, tangent);
				float dPt = (-vt) / contact[index].tangent_mass;

				// Compute friction impulse
				u = sqrt(A->material.uk*A->material.uk + B->material.uk*B->material.uk);
				float maxPt = u * contact[index].normal_impulse;

				// Clamp friction
				float oldTangentImpulse = contact[index].tangent_impulse;
				contact[index].tangent_impulse = clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
				dPt = contact[index].tangent_impulse - oldTangentImpulse;

				// Apply contact impulse
				float Pt[2]; Pt[0] = dPt*tangent[0]; Pt[1] = dPt*tangent[1];

				A->v[0] = A->v[0] - A->inv_mass * Pt[0]; A->v[1] = A->v[1] - A->inv_mass * Pt[1];
				A->w = A->w - A->inv_inertia*v_cross(rA, Pt);

				B->v[0] = B->v[0] + B->inv_mass*Pt[0]; B->v[1] = B->v[1] + B->inv_mass*Pt[1]; // impulse on B's v
				B->w = B->w + B->inv_inertia*v_cross(rB, Pt); // impulse on B's w
		}
	}
}

void positionalcorrection(Object * A, Object * B) {
	float tempt_vector[2]; // temporary vector for calculus
	int indexA, indexB;
	float depth;
	const float percent = 0.2;

	for (indexB = 0; indexB < B->shape->vertex_num; indexB++) {
		depth = find_penetration_depth(A, B, &indexB, &indexA);
		if (depth < -slop) {
			depth = -depth;
			v_mult(A->shape->normal[indexA], depth*percent / (A->inv_mass + B->inv_mass), tempt_vector);
			float tempttempt_vector[2];
			v_mult(tempt_vector, -A->inv_mass, tempttempt_vector);
			v_sum(A->x, tempttempt_vector, A->x);
			v_mult(tempt_vector, B->inv_mass, tempttempt_vector);
			v_sum(B->x, tempttempt_vector, B->x);
		}
	}
}

void apply_force(Object *A) {
	float tempt[2];
	float dt = 1 / fpss;

	v_mult(A->F, A->inv_mass, tempt);  // tempt=F/m
	v_mult(tempt, dt, tempt); // tempt=dt * F/m
	v_sum(A->v, tempt, A->v); // v=v+dt*F/m
}

void movement(Object * A) {
	float tempt[2];
	float dt = 1 / fpss;

	v_mult(A->v, dt, tempt); // tempt=v*dt
	v_sum(A->x, tempt, A->x); // x=x+v*dt
	
	A->w = A->w + A->alp*dt;
	A->th = A->th + A->w*dt;
	float rotmat[2][2];
	set_rot_mat(rotmat, &A->th);
	int i;
		switch (A->shape->number) {
		case 1: {
			float vertex[4][2];
			vertex[0][0] = Length / 2; vertex[0][1] = Length / 2;
			vertex[1][0] = -Length / 2; vertex[1][1] = Length / 2;
			vertex[2][0] = -Length / 2; vertex[2][1] = -Length / 2;
			vertex[3][0] = Length / 2; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			float normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			break;
		}
		case 2: {
			float vertex[4][2];
			vertex[0][0] = Length; vertex[0][1] = Length / 2;
			vertex[1][0] = -Length; vertex[1][1] = Length / 2;
			vertex[2][0] = -Length; vertex[2][1] = -Length / 2;
			vertex[3][0] = Length; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			float normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			break;
		}
		case 3: {
			float vertex[4][2];
			vertex[0][0] = 3 * Length / 2; vertex[0][1] = Length / 2;
			vertex[1][0] = -3 * Length / 2; vertex[1][1] = Length / 2;
			vertex[2][0] = -3 * Length / 2; vertex[2][1] = -Length / 2;
			vertex[3][0] = 3 * Length / 2; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			float normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			break;
		}
		case 4: {
			float vertex[3][2];
			vertex[0][0] = 0; vertex[0][1] = -Length / sqrt(3);
			vertex[1][0] = -Length / 2; vertex[1][1] = Length / (2 * sqrt(3));
			vertex[2][0] = Length / 2; vertex[2][1] = Length / (2 * sqrt(3));
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			float normal[3][2];
			normal[0][0] = -sqrt(3) / 2; normal[0][1] = -1.0 / 2;
			normal[1][0] = 0; normal[1][1] = 1;
			normal[2][0] = sqrt(3) / 2; normal[2][1] = -1.0 / 2;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			break;
		}
		case 5: {
			float vertex[3][2];
			vertex[0][0] = -Length / 3; vertex[0][1] = -2 * Length / 3;
			vertex[1][0] = -Length / 3; vertex[1][1] = Length / 3;
			vertex[2][0] = 2 * Length / 3; vertex[2][1] = Length / 3;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			float normal[3][2];
			normal[0][0] = -1; normal[0][1] = 0;
			normal[1][0] = 0; normal[1][1] = 1;
			normal[2][0] = sqrt(2) / 2; normal[2][1] = -sqrt(2) / 2;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			break;
		}
		case 6: {
			float vertex[4][2];
			vertex[0][0] = Length / 2; vertex[0][1] = -5 * Length / (6 * sqrt(3));
			vertex[1][0] = -Length / 2; vertex[1][1] = -5 * Length / (6 * sqrt(3));
			vertex[2][0] = -Length; vertex[2][1] = 2 * Length / (3 * sqrt(3));
			vertex[3][0] = Length; vertex[3][1] = 2 * Length / (3 * sqrt(3));
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			float normal[4][2];
			normal[0][0] = 0; normal[0][1] = -1;
			normal[1][0] = -sqrt(3) / 2.0; normal[1][1] = -1.0 / 2.0;
			normal[2][0] = 0; normal[2][1] = 1;
			normal[3][0] = sqrt(3) / 2.0; normal[3][1] = -1.0 / 2.0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			break;
		}
		case 8: {
			float vertex[6][2];
			vertex[0][0] = Length / 2.0; vertex[0][1] = -sqrt(3)*Length / 2.0;
			vertex[1][0] = -Length / 2.0; vertex[1][1] = -sqrt(3)*Length / 2.0;
			vertex[2][0] = -Length; vertex[2][1] = 0;
			vertex[3][0] = -Length / 2.0; vertex[3][1] = sqrt(3)*Length / 2.0;
			vertex[4][0] = Length / 2.0; vertex[4][1] = sqrt(3)*Length / 2.0;
			vertex[5][0] = Length; vertex[5][1] = 0;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			mat_v_product(rotmat, vertex[5], A->shape->vertex[5]);
			float normal[6][2];
			normal[0][0] = 0; normal[0][1] = -1;
			normal[1][0] = -sqrt(3) / 2.0; normal[1][1] = -1.0 / 2.0;
			normal[2][0] = -sqrt(3) / 2.0; normal[2][1] = 1.0 / 2.0;
			normal[3][0] = 0; normal[3][1] = 1;
			normal[4][0] = sqrt(3) / 2.0; normal[4][1] = 1.0 / 2.0;
			normal[5][0] = sqrt(3) / 2.0; normal[5][1] = -1.0 / 2.0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			mat_v_product(rotmat, normal[5], A->shape->normal[5]);
			break;
		}
		case 9: {
			float vertex[5][2];
			vertex[0][0] = 2.0*Length / 21.0; vertex[0][1] = -23.0*Length / 21.0;
			vertex[1][0] = -19.0*Length / 21.0; vertex[1][1] = -23.0*Length / 21.0;
			vertex[2][0] = -19.0*Length / 21.0; vertex[2][1] = 19.0*Length / 21.0;
			vertex[3][0] = 23.0*Length / 21.0; vertex[3][1] = 19.0*Length / 21.0;
			vertex[4][0] = 23.0*Length / 21.0; vertex[4][1] = -2.0*Length / 21.0;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			float normal[5][2];
			normal[0][0] = 0; normal[0][1] = -1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = 1;
			normal[3][0] = 1; normal[3][1] = 0;
			normal[4][0] = sqrt(2) / 2.0; normal[4][1] = -sqrt(2) / 2.0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			break;
		}
		case 80: {
			float vertex[11][2];
			vertex[0][0] = 12.036667*Length / 40.0; vertex[0][1] = -33.022701*Length / 40.0;
			vertex[1][0] = -29.963333*Length / 40.0; vertex[1][1] = -26.022701*Length / 40.0;
			vertex[2][0] = -48.963333*Length / 40.0; vertex[2][1] = 6.977299*Length / 40.0;
			vertex[3][0] = -49.963333*Length / 40.0; vertex[3][1] = 14.977299*Length / 40.0;
			vertex[4][0] = -41.963333*Length / 40.0; vertex[4][1] = 26.977299*Length / 40.0;
			vertex[5][0] = -22.963333*Length / 40.0; vertex[5][1] = 34.977299*Length / 40.0;
			vertex[6][0] = 34.036667*Length / 40.0; vertex[6][1] = 20.977299*Length / 40.0;
			vertex[7][0] = 42.036667*Length / 40.0; vertex[7][1] = 12.977299*Length / 40.0;
			vertex[8][0] = 49.036667*Length / 40.0; vertex[8][1] = -1.022701*Length / 40.0;
			vertex[9][0] = 49.036667*Length / 40.0; vertex[9][1] = -15.022701*Length / 40.0;
			vertex[10][0] = 41.036667*Length / 40.0; vertex[10][1] = -27.022701*Length / 40.0;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			mat_v_product(rotmat, vertex[5], A->shape->vertex[5]);
			mat_v_product(rotmat, vertex[6], A->shape->vertex[6]);
			mat_v_product(rotmat, vertex[7], A->shape->vertex[7]);
			mat_v_product(rotmat, vertex[8], A->shape->vertex[8]);
			mat_v_product(rotmat, vertex[9], A->shape->vertex[9]);
			mat_v_product(rotmat, vertex[10], A->shape->vertex[10]);
			float normal[11][2];
			normal[0][0] = -0.164399; normal[0][1] = -0.986394;
			normal[1][0] = -0.866622; normal[1][1] = -0.498964;
			normal[2][0] = -0.992278; normal[2][1] = -0.124035;
			normal[3][0] = -0.832050; normal[3][1] = 0.554700;
			normal[4][0] = -0.388057; normal[4][1] = 0.921635;
			normal[5][0] = 0.238525; normal[5][1] = 0.971136;
			normal[6][0] = 0.707107; normal[6][1] = 0.707107;
			normal[7][0] = 0.894427; normal[7][1] = 0.447214;
			normal[8][0] = 1.000000; normal[8][1] = 0.000000;
			normal[9][0] = 0.832050; normal[9][1] = -0.554700;
			normal[10][0] = 0.202606; normal[10][1] = -0.979260;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			mat_v_product(rotmat, normal[5], A->shape->normal[5]);
			mat_v_product(rotmat, normal[6], A->shape->normal[6]);
			mat_v_product(rotmat, normal[7], A->shape->normal[7]);
			mat_v_product(rotmat, normal[8], A->shape->normal[8]);
			mat_v_product(rotmat, normal[9], A->shape->normal[9]);
			mat_v_product(rotmat, normal[10], A->shape->normal[10]);
			break;
		}
		case 81: {
			float vertex[6][2];
			vertex[0][0] = -0.558511*Length; vertex[0][1] = -0.759908*Length;
			vertex[1][0] = -0.408511*Length; vertex[1][1] = 0.765092*Length;
			vertex[2][0] = 0.491489*Length; vertex[2][1] = 0.765092*Length;
			vertex[3][0] = 0.516489*Length; vertex[3][1] = -0.509908*Length;
			vertex[4][0] = 0.466489*Length; vertex[4][1] = -0.609908*Length;
			vertex[5][0] = 0.016489*Length; vertex[5][1] = -0.759908*Length;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			mat_v_product(rotmat, vertex[5], A->shape->vertex[5]);
			float normal[6][2];
			normal[0][0] = -0.995197; normal[0][1] = 0.097888;
			normal[1][0] = -0.000000; normal[1][1] = 1.000000;
			normal[2][0] = 0.999808; normal[2][1] = 0.019604;
			normal[3][0] = 0.894427; normal[3][1] = -0.447214;
			normal[4][0] = 0.316228; normal[4][1] = -0.948683;
			normal[5][0] = -0.000000; normal[5][1] = -1.000000;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			mat_v_product(rotmat, normal[5], A->shape->normal[5]);
			break;
		}
		case 95: {
			float vertex[5][2];
			vertex[0][0] = -1.007442*Length; vertex[0][1] = -2.040490*Length;
			vertex[1][0] = -0.157442*Length; vertex[1][1] = 3.309510*Length;
			vertex[2][0] = 0.617558*Length; vertex[2][1] = 3.384510*Length;
			vertex[3][0] = 0.617558*Length; vertex[3][1] = -2.865490*Length;
			vertex[4][0] = 0.217558*Length; vertex[4][1] = -2.940490*Length;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			float normal[5][2];
			normal[0][0] = -0.987613; normal[0][1] = 0.156910;
			normal[1][0] = -0.096324; normal[1][1] = 0.995350;
			normal[2][0] = 1.000000; normal[2][1] = 0.000000;
			normal[3][0] = 0.184289; normal[3][1] = -0.982872;
			normal[4][0] = -0.592076; normal[4][1] = -0.805882;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			break;
		}
		case 96: {
			float vertex[6][2];
			vertex[0][0] = -0.877352*Length; vertex[0][1] = -3.056292*Length;
			vertex[1][0] = -0.877352*Length; vertex[1][1] = 3.393708*Length;
			vertex[2][0] = 0.647648*Length; vertex[2][1] = 3.168708*Length;
			vertex[3][0] = 1.122648*Length; vertex[3][1] = -2.356292*Length;
			vertex[4][0] = 0.872648*Length; vertex[4][1] = -2.806292*Length;
			vertex[5][0] = -0.252352*Length; vertex[5][1] = -3.156292*Length;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			mat_v_product(rotmat, vertex[5], A->shape->vertex[5]);
			float normal[6][2];
			normal[0][0] = -1.000000; normal[0][1] = 0.000000;
			normal[1][0] = 0.145961; normal[1][1] = 0.989290;
			normal[2][0] = 0.996325; normal[2][1] = 0.085657;
			normal[3][0] = 0.874157; normal[3][1] = -0.485643;
			normal[4][0] = 0.297067; normal[4][1] = -0.954857;
			normal[5][0] = -0.157991; normal[5][1] = -0.987441;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			mat_v_product(rotmat, normal[5], A->shape->normal[5]);
			break;
		}
		case 97: {
			float vertex[5][2];
			vertex[0][0] = -1.296671*Length; vertex[0][1] = -2.968418*Length;
			vertex[1][0] = -1.296671*Length; vertex[1][1] = 3.281582*Length;
			vertex[2][0] = 1.278329*Length; vertex[2][1] = 3.406582*Length;
			vertex[3][0] = 1.278329*Length; vertex[3][1] = -3.243418*Length;
			vertex[4][0] = -0.321671*Length; vertex[4][1] = -3.543418*Length;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			float normal[5][2];
			normal[0][0] = -1.000000; normal[0][1] = 0.000000;
			normal[1][0] = -0.048487; normal[1][1] = 0.998824;
			normal[2][0] = 1.000000; normal[2][1] = 0.000000;
			normal[3][0] = 0.184289; normal[3][1] = -0.982872;
			normal[4][0] = -0.507985; normal[4][1] = -0.861366;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			break;
		}
		case 98: {
			float vertex[5][2];
			vertex[0][0] = 77.5*Length / 40.0; vertex[0][1] = -4459.0*Length / (219.0*40.0);
			vertex[1][0] = -77.5*Length / 40.0; vertex[1][1] = -4459.0*Length / (219.0*40.0);
			vertex[2][0] = -77.5*Length / 40.0; vertex[2][1] = 1174.0 * Length / (219.0*40.0);
			vertex[3][0] = 0.0; vertex[3][1] = 8243.0 * Length / (219.0*40.0);
			vertex[4][0] = 77.5*Length / 40.0; vertex[4][1] = 1174.0 * Length / (219.0*40.0);
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			float normal[5][2];
			normal[0][0] = 0.0; normal[0][1] = -1.0;
			normal[1][0] = -1.0; normal[1][1] = 0.0;
			normal[2][0] = -0.48516; normal[2][1] = 0.87442;
			normal[3][0] = 0.48516; normal[3][1] = 0.87442;
			normal[4][0] = 1.0; normal[4][1] = 0.0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			break;
		}
		case 99: {
			float vertex[4][2];
			vertex[0][0] = 7 * Length; vertex[0][1] = 2 * Length;
			vertex[1][0] = -7 * Length; vertex[1][1] = 2 * Length;
			vertex[2][0] = -7 * Length; vertex[2][1] = -2 * Length;
			vertex[3][0] = 7 * Length; vertex[3][1] = -2 * Length;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			float normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			break;
		}
		}
}

void gravity(Object * A) {
	if (A->inv_mass != 0) A->F[1] = A->F[1] + g*(1 / A->inv_mass);
}

void force_zero(Object * A) {
	A->F[0] = 0;
	A->F[1] = 0;
}

void v_sum(float v1[2], float v2[2], float v_out[2]) {
	v_out[0] = v1[0] + v2[0];
	v_out[1] = v1[1] + v2[1];
}

void v_sub(float v1[2], float v2[2], float v_out[2]) { // v_out=v1-v2
	v_out[0] = v1[0] - v2[0];
	v_out[1] = v1[1] - v2[1];
}

float v_dot(float v1[2], float v2[2]) { // v1 * v2
	return v1[0] * v2[0] + v1[1] * v2[1];
}

float v_cross(float v1[2], float v2[2]) { //v1 X v2
	return v1[0] * v2[1] - v1[1] * v2[0];
}

void v_cross_k(float vec[2], float k, float v_out[2]) { // <vec[0], vec[1], 0> X <0,0,k>
	v_out[0] = k*vec[1];
	v_out[1] = -vec[0] * k;
}

void v_cross_k_inv(float k, float vec[2], float v_out[2]) { // <0,0,k> X <vec[0], vec[1], 0>
	v_out[0] = -k*vec[1];
	v_out[1] = vec[0] * k;
}

float v_magnitude(float v[2]) {
	return sqrt(v[0] * v[0] + v[1] * v[1]);
}

void v_normalization(float v_in[2], float v_out[2]) {
	v_out[0] = v_in[0] / v_magnitude(v_in);
	v_out[1] = v_in[1] / v_magnitude(v_in);
}

void v_normalization_s(float v_in[2]) {
	float mag = v_magnitude(v_in);
	v_in[0] = v_in[0] / mag;
	v_in[1] = v_in[1] / mag;
}

void mat_v_product(float mat[2][2], float v[2], float v_out[2]) { // matrix * vector
	v_out[0] = mat[0][0] * v[0] + mat[0][1] * v[1];
	v_out[1] = mat[1][0] * v[0] + mat[1][1] * v[1];
}

void set_rot_mat(float mat[2][2], float *radius) {
	mat[0][0] = cos(*radius);
	mat[0][1] = -sin(*radius);
	mat[1][0] = sin(*radius);
	mat[1][1] = cos(*radius);
/*	if (cos(*radius) > 0.9999) {
		*radius = 0;
		mat[0][0] = 1;
		mat[0][1] = 0;
		mat[1][0] = 0;
		mat[1][1] = 1;
	}
	else if (cos(*radius) < -0.9999) {
		*radius = pi;
		mat[0][0] = -1;
		mat[0][1] = 0;
		mat[1][0] = 0;
		mat[1][1] = -1;
	}
	else if (sin(*radius) > 0.9999) {
		*radius = pi/2;
		mat[0][0] = 0;
		mat[0][1] = -1;
		mat[1][0] = 1;
		mat[1][1] = 0;
	}
	else if (sin(*radius) < -0.9999) {
		*radius = 3*pi/2;
		mat[0][0] = 0;
		mat[0][1] = 1;
		mat[1][0] = -1;
		mat[1][1] = 0;
	}*/
}

void find_normal_v(float v[2], float dir[2], float v_out[2]) { // find vector along some direction
	float magnitude;
	float normalization_vector[2];
	v_normalization(dir, normalization_vector);
	magnitude = v_dot(v, normalization_vector);
	v_out[0] = magnitude*normalization_vector[0];
	v_out[1] = magnitude*normalization_vector[1];
}

void find_perpend_v(float v[2], float dir[2], float v_out[2]) { // find vector perpendicular to some direction
	float normal_vector[2];
	find_normal_v(v, dir, normal_vector);
	v_out[0] = v[0] - normal_vector[0];
	v_out[1] = v[1] - normal_vector[1];
}

void v_mult(float v[2], float n, float v_out[2]) {
	v_out[0] = n*v[0];
	v_out[1] = n*v[1];
}

float minimum(float a, float b) {
	if (a >= b) return b;
	else return a;
}

float maximum(float a, float b) {
	if (a <= b) return b;
	else return a;
}

float absolute(float a) {
	if (a >= 0) return a;
	else return -a;
}

void set_shape(Shape * shape, int n) {
	enum polygon { oneblock = 1, twoblock = 2, threeblock = 3, triangle = 4, right_triangle, trapezoid, parallelogram, hexagon, polygon1, stone = 80,cat, ground_hill_left = 95, ground_hill_right, ground_hill_middle, ground_cone_small = 98, ground = 99, };
	if (n == oneblock) {
		int i;
		shape->vertex_num = 4;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = Length / 2; shape->vertex[0][1] = Length / 2;
		shape->vertex[1][0] = -Length / 2; shape->vertex[1][1] = Length / 2;
		shape->vertex[2][0] = -Length / 2; shape->vertex[2][1] = -Length / 2;
		shape->vertex[3][0] = Length / 2; shape->vertex[3][1] = -Length / 2;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = 1;
		shape->normal[1][0] = -1; shape->normal[1][1] = 0;
		shape->normal[2][0] = 0; shape->normal[2][1] = -1;
		shape->normal[3][0] = 1; shape->normal[3][1] = 0;

		shape->volume = Length*Length;
		shape->inertia_constant = 1 / 6.0;
		shape->number = 1;

		shape->max_distance = Length / sqrt(2);

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
		/*
		n[0]
		|
		vert[1]----vert[0]
		|				|
		|				|
		n[1]-|				|-n[3]
		|				|
		vert[2]----vert[3]
		|
		n[2]
		*/
	}
	else if (n == twoblock) {
		int i;
		shape->vertex_num = 4;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = Length; shape->vertex[0][1] = Length / 2;
		shape->vertex[1][0] = -Length; shape->vertex[1][1] = Length / 2;
		shape->vertex[2][0] = -Length; shape->vertex[2][1] = -Length / 2;
		shape->vertex[3][0] = Length; shape->vertex[3][1] = -Length / 2;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = 1;
		shape->normal[1][0] = -1; shape->normal[1][1] = 0;
		shape->normal[2][0] = 0; shape->normal[2][1] = -1;
		shape->normal[3][0] = 1; shape->normal[3][1] = 0;

		shape->volume = 2 * Length*Length;
		shape->inertia_constant = 5 / 24.0;
		shape->number = 2;

		shape->max_distance = 0.5*Length*sqrt(5);

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
		/*
					n[0]
					|
		vert[1]-----------------vert[0]
		|			|			|
		|			|			|
   n[1]-|			|			|-n[3]
		|			|			|
		vert[2]-----------------vert[3]
					|
					n[2]
		*/
	}
	else if (n == threeblock) {
		int i;
		shape->vertex_num = 4;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = 3 * Length / 2; shape->vertex[0][1] = Length / 2;
		shape->vertex[1][0] = -3 * Length / 2; shape->vertex[1][1] = Length / 2;
		shape->vertex[2][0] = -3 * Length / 2; shape->vertex[2][1] = -Length / 2;
		shape->vertex[3][0] = 3 * Length / 2; shape->vertex[3][1] = -Length / 2;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = 1;
		shape->normal[1][0] = -1; shape->normal[1][1] = 0;
		shape->normal[2][0] = 0; shape->normal[2][1] = -1;
		shape->normal[3][0] = 1; shape->normal[3][1] = 0;

		shape->volume = 3 * Length*Length;
		shape->inertia_constant = 5 / 18.0;
		shape->number = 3;

		shape->max_distance = 0.5*Length*sqrt(10);

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
		/*
		n[0]
		|
		vert[1]-------------------------vert[0]
		|			|			|			|
		|			|			|			|
		n[1]-|			|			|			|-n[3]
		|			|			|			|
		vert[2]-------------------------vert[3]
		|
		n[2]
		*/
	}
	else if (n == triangle) {
		int i;
		shape->vertex_num = 3;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = 0; shape->vertex[0][1] = -Length/sqrt(3);
		shape->vertex[1][0] = -Length / 2; shape->vertex[1][1] = Length / (2 * sqrt(3));
		shape->vertex[2][0] = Length / 2; shape->vertex[2][1] = Length / (2 * sqrt(3));

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = -sqrt(3) / 2; shape->normal[0][1] = -1.0 / 2;
		shape->normal[1][0] = 0; shape->normal[1][1] = 1;
		shape->normal[2][0] = sqrt(3) / 2; shape->normal[2][1] = -1.0 / 2;

		shape->volume = sqrt(3) * Length*Length / 4;
		shape->inertia_constant = sqrt(3) / 9.0;
		shape->number = 4;

		shape->max_distance = Length / sqrt(3);

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
		/*
		                vert[0]
	                  /        \    n[2]
		    n[0]     /	        \  /
				 \  /            \
                   /              \
				vert[1]--------vert[2]
				          |
				         n[1]
		*/
	}
	else if (n == right_triangle) {
		int i;
		shape->vertex_num = 3;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = -Length / 3; shape->vertex[0][1] = -2 * Length / 3;
		shape->vertex[1][0] = -Length / 3; shape->vertex[1][1] = Length / 3;
		shape->vertex[2][0] = 2 * Length / 3; shape->vertex[2][1] = Length / 3;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = -1; shape->normal[0][1] = 0;
		shape->normal[1][0] = 0; shape->normal[1][1] = 1;
		shape->normal[2][0] = sqrt(2) / 2; shape->normal[2][1] = -sqrt(2) / 2;

		shape->volume = Length*Length / 2;
		shape->inertia_constant = 2.0 / 9.0;
		shape->number = 5;

		shape->max_distance = sqrt(5) * Length / 3;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
		/*
	      	vert[0]
              |     \    n[2]
		      |      \  /
		n[0]--|       \/
              |        \
		vert[1]--------vert[2]
		           |
				   n[1]
		*/
	}
	else if (n == trapezoid) {
		int i;
		shape->vertex_num = 4;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = Length / 2; shape->vertex[0][1] = -5 * Length / (6 * sqrt(3));
		shape->vertex[1][0] = -Length / 2; shape->vertex[1][1] = -5 * Length / (6 * sqrt(3));
		shape->vertex[2][0] = -Length; shape->vertex[2][1] = 2 * Length / (3 * sqrt(3));
		shape->vertex[3][0] = Length; shape->vertex[3][1] = 2 * Length / (3 * sqrt(3));

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = -1;
		shape->normal[1][0] = -sqrt(3) / 2.0; shape->normal[1][1] = -1.0 / 2.0;
		shape->normal[2][0] = 0; shape->normal[2][1] = 1;
		shape->normal[3][0] = sqrt(3) / 2.0; shape->normal[3][1] = -1.0 / 2.0;

		shape->volume = 3 * sqrt(3)*Length*Length / 4.0;
		shape->inertia_constant = 103 * sqrt(3) / (27 * 27);
		shape->number = 6;

		shape->max_distance = sqrt(93)*Length / 9.0;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
		/*
		                n[0]
		                 |
	     	vert[1]---------------vert[0]
		   /						\		
    n[1]  /							 \	n[3]
        \/							  \/
		/							   \		
		vert[2]-------------------------vert[3]
		                 |
		                n[2]
		*/
	}
	else if (n == parallelogram) {
		int i;
		shape->vertex_num = 4;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = 3 * Length / 4.0; shape->vertex[0][1] = -sqrt(3)*Length / 4.0;
		shape->vertex[1][0] = -Length / 4.0; shape->vertex[1][1] = -sqrt(3)*Length / 4.0;
		shape->vertex[2][0] = -3 * Length / 4.0; shape->vertex[2][1] = sqrt(3)*Length / 4.0;
		shape->vertex[3][0] = Length; shape->vertex[3][1] = sqrt(3)*Length / 4.0;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = -1;
		shape->normal[1][0] = -sqrt(3) / 2; shape->normal[1][1] = -1.0 / 2.0;
		shape->normal[2][0] = 0; shape->normal[2][1] = 1;
		shape->normal[3][0] = sqrt(3) / 2.0; shape->normal[3][1] = 1.0 / 2.0;

		shape->volume = sqrt(3)*Length*Length / 2.0;
		shape->inertia_constant = 5.0*sqrt(3) / 18.0;
		shape->number = 7;

		shape->max_distance = sqrt(3)*Length / 2.0;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
		/*
		                  n[0]
		                   |
	     	vert[1]--------------------vert[0]
	        	/						 /
		n[1]  /							/\
	     	\/					       /  n[3]
	     	/						  /   
		vert[2]------------------vert[3]
		                   |
					   	n[2]
		*/
	}
	else if (n == hexagon) {
		int i;
		shape->vertex_num = 6;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = Length/2.0; shape->vertex[0][1] = -sqrt(3)*Length / 2.0;
		shape->vertex[1][0] = -Length / 2.0; shape->vertex[1][1] = -sqrt(3)*Length / 2.0;
		shape->vertex[2][0] = -Length; shape->vertex[2][1] = 0;
		shape->vertex[3][0] = -Length/2.0; shape->vertex[3][1] = sqrt(3)*Length / 2.0;
		shape->vertex[4][0] = Length / 2.0; shape->vertex[4][1] = sqrt(3)*Length / 2.0;
		shape->vertex[5][0] = Length; shape->vertex[5][1] = 0;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = -1;
		shape->normal[1][0] = -sqrt(3) / 2.0; shape->normal[1][1] = -1.0 / 2.0;
		shape->normal[2][0] = -sqrt(3) / 2.0; shape->normal[2][1] = 1.0 / 2.0;
		shape->normal[3][0] = 0; shape->normal[3][1] = 1;
		shape->normal[4][0] = sqrt(3) / 2.0; shape->normal[4][1] = 1.0 / 2.0;
		shape->normal[5][0] = sqrt(3) / 2.0; shape->normal[5][1] = -1.0 / 2.0;

		shape->volume = 3.0*sqrt(3)*Length*Length / 2.0;
		shape->inertia_constant = 5.0*sqrt(3) / 54.0;
		shape->number = 8;

		shape->max_distance = Length;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
		/*
		            n[0]
					|
		vert[1]-----------vert[0]
		/                       \
     vert[2]                  vert[5]
	     \                     /
	    vert[3]-----------vert[4]
		            |
					n[3]
		*/
	}
	else if (n == polygon1) {
		int i;
		shape->vertex_num = 5;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = 2.0*Length/21.0; shape->vertex[0][1] = -23.0*Length / 21.0;
		shape->vertex[1][0] = -19.0*Length / 21.0; shape->vertex[1][1] = -23.0*Length / 21.0;
		shape->vertex[2][0] = -19.0*Length / 21.0; shape->vertex[2][1] = 19.0*Length / 21.0;
		shape->vertex[3][0] = 23.0*Length / 21.0; shape->vertex[3][1] = 19.0*Length / 21.0;
		shape->vertex[4][0] = 23.0*Length / 21.0; shape->vertex[4][1] = -2.0*Length / 21.0;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = -1;
		shape->normal[1][0] = -1; shape->normal[1][1] = 0;
		shape->normal[2][0] = 0; shape->normal[2][1] = 1;
		shape->normal[3][0] = 1; shape->normal[3][1] = 0;
		shape->normal[4][0] = sqrt(2) / 2.0; shape->normal[4][1] = -sqrt(2) / 2.0;

		shape->volume = 7.0*Length*Length / 2.0;
		shape->inertia_constant = 674.0 / 3087.0;
		shape->number = 9;

		shape->max_distance = sqrt(890)*Length / 21.0;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
		/*
		vert[1]-----vert[0]
		   |               \
		   |              vert[4]
		   |                  |
		vert[2]-----------vert[3]
		*/
	}
	else if (n == stone) {
		int i;
		shape->vertex_num = 11;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = 12.036667*Length / 40.0; shape->vertex[0][1] = -33.022701*Length / 40.0;
		shape->vertex[1][0] = -29.963333*Length / 40.0; shape->vertex[1][1] = -26.022701*Length / 40.0;
		shape->vertex[2][0] = -48.963333*Length / 40.0; shape->vertex[2][1] = 6.977299*Length / 40.0;
		shape->vertex[3][0] = -49.963333*Length / 40.0; shape->vertex[3][1] = 14.977299*Length / 40.0;
		shape->vertex[4][0] = -41.963333*Length / 40.0; shape->vertex[4][1] = 26.977299*Length / 40.0;
		shape->vertex[5][0] = -22.963333*Length / 40.0; shape->vertex[5][1] = 34.977299*Length / 40.0;
		shape->vertex[6][0] = 34.036667*Length / 40.0; shape->vertex[6][1] = 20.977299*Length / 40.0;
		shape->vertex[7][0] = 42.036667*Length / 40.0; shape->vertex[7][1] = 12.977299*Length / 40.0;
		shape->vertex[8][0] = 49.036667*Length / 40.0; shape->vertex[8][1] = -1.022701*Length / 40.0;
		shape->vertex[9][0] = 49.036667*Length / 40.0; shape->vertex[9][1] = -15.022701*Length / 40.0;
		shape->vertex[10][0] = 41.036667*Length / 40.0; shape->vertex[10][1] = -27.022701*Length / 40.0;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = -0.164399; shape->normal[0][1] = -0.986394;
		shape->normal[1][0] = -0.866622; shape->normal[1][1] = -0.498964;
		shape->normal[2][0] = -0.992278; shape->normal[2][1] = -0.124035;
		shape->normal[3][0] = -0.832050; shape->normal[3][1] = 0.554700;
		shape->normal[4][0] = -0.388057; shape->normal[4][1] = 0.921635;
		shape->normal[5][0] = 0.238525; shape->normal[5][1] = 0.971136;
		shape->normal[6][0] = 0.707107; shape->normal[6][1] = 0.707107;
		shape->normal[7][0] = 0.894427; shape->normal[7][1] = 0.447214;
		shape->normal[8][0] = 1.000000; shape->normal[8][1] = 0.000000;
		shape->normal[9][0] = 0.832050; shape->normal[9][1] = -0.554700;
		shape->normal[10][0] = 0.202606; shape->normal[10][1] = -0.979260;

		shape->volume = 4977.5*Length*Length/1600.0;
		shape->inertia_constant = 674.0 / 3087.0;
		shape->number = 80;

		shape->max_distance = 52.2*Length / 40.0;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
	}
	else if (n == cat) {
		int i;
		shape->vertex_num = 6;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = -0.558511*Length; shape->vertex[0][1] = -0.759908*Length;
		shape->vertex[1][0] = -0.408511*Length; shape->vertex[1][1] = 0.765092*Length;
		shape->vertex[2][0] = 0.491489*Length; shape->vertex[2][1] = 0.765092*Length;
		shape->vertex[3][0] = 0.516489*Length; shape->vertex[3][1] = -0.509908*Length;
		shape->vertex[4][0] = 0.466489*Length; shape->vertex[4][1] = -0.609908*Length;
		shape->vertex[5][0] = 0.016489*Length; shape->vertex[5][1] = -0.759908*Length;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = -0.995197; shape->normal[0][1] = 0.097888;
		shape->normal[1][0] = -0.000000; shape->normal[1][1] = 1.000000;
		shape->normal[2][0] = 0.999808; shape->normal[2][1] = 0.019604;
		shape->normal[3][0] = 0.894427; shape->normal[3][1] = -0.447214;
		shape->normal[4][0] = 0.316228; shape->normal[4][1] = -0.948683;
		shape->normal[5][0] = -0.000000; shape->normal[5][1] = -1.000000;

		shape->volume = 1.46f*Length*Length;
		shape->inertia_constant = 1.0;
		shape->number = 81;

		shape->max_distance = 0.944*Length;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = -1;
	}
	else if (n == ground_hill_left) {
		int i;
		shape->vertex_num = 5;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = -1.007442*Length; shape->vertex[0][1] = -2.040490*Length;
		shape->vertex[1][0] = -0.157442*Length; shape->vertex[1][1] = 3.309510*Length;
		shape->vertex[2][0] = 0.617558*Length; shape->vertex[2][1] = 3.384510*Length;
		shape->vertex[3][0] = 0.617558*Length; shape->vertex[3][1] = -2.865490*Length;
		shape->vertex[4][0] = 0.217558*Length; shape->vertex[4][1] = -2.940490*Length;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = -0.987613; shape->normal[0][1] = 0.156910;
		shape->normal[1][0] = -0.096324; shape->normal[1][1] = 0.995350;
		shape->normal[2][0] = 1.000000; shape->normal[2][1] = 0.000000;
		shape->normal[3][0] = 0.184289; shape->normal[3][1] = -0.982872;
		shape->normal[4][0] = -0.592076; shape->normal[4][1] = -0.805882;

		shape->volume = 7.345f*Length*Length;
		shape->inertia_constant = 0.2f; // assumption value
		shape->number = 95;

		shape->max_distance = 3.45*Length;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = 1;
	}
	else if (n == ground_hill_right) {
		int i;
		shape->vertex_num = 6;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = -0.877352*Length; shape->vertex[0][1] = -3.056292*Length;
		shape->vertex[1][0] = -0.877352*Length; shape->vertex[1][1] = 3.393708*Length;
		shape->vertex[2][0] = 0.647648*Length; shape->vertex[2][1] = 3.168708*Length;
		shape->vertex[3][0] = 1.122648*Length; shape->vertex[3][1] = -2.356292*Length;
		shape->vertex[4][0] = 0.872648*Length; shape->vertex[4][1] = -2.806292*Length;
		shape->vertex[5][0] = -0.252352*Length; shape->vertex[5][1] = -3.156292*Length;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = -1.000000; shape->normal[0][1] = 0.000000;
		shape->normal[1][0] = 0.145961; shape->normal[1][1] = 0.989290;
		shape->normal[2][0] = 0.996325; shape->normal[2][1] = 0.085657;
		shape->normal[3][0] = 0.874157; shape->normal[3][1] = -0.485643;
		shape->normal[4][0] = 0.297067; shape->normal[4][1] = -0.954857;
		shape->normal[5][0] = -0.157991; shape->normal[5][1] = -0.987441;

		shape->volume = 11.1375f*Length*Length;
		shape->inertia_constant = 0.2f; // assumption value
		shape->number = 96;

		shape->max_distance = 3.51f*Length;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = 1;
	}
	else if (n == ground_hill_middle) {
		int i;
		shape->vertex_num = 5;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = -1.296671*Length; shape->vertex[0][1] = -2.968418*Length;
		shape->vertex[1][0] = -1.296671*Length; shape->vertex[1][1] = 3.281582*Length;
		shape->vertex[2][0] = 1.278329*Length; shape->vertex[2][1] = 3.406582*Length;
		shape->vertex[3][0] = 1.278329*Length; shape->vertex[3][1] = -3.243418*Length;
		shape->vertex[4][0] = -0.321671*Length; shape->vertex[4][1] = -3.543418*Length;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = -1.000000; shape->normal[0][1] = 0.000000;
		shape->normal[1][0] = -0.048487; shape->normal[1][1] = 0.998824;
		shape->normal[2][0] = 1.000000; shape->normal[2][1] = 0.000000;
		shape->normal[3][0] = 0.184289; shape->normal[3][1] = -0.982872;
		shape->normal[4][0] = -0.507985; shape->normal[4][1] = -0.861366;

		shape->volume = 17.215f*Length*Length;
		shape->inertia_constant = 0.2f; // assumption value
		shape->number = 97;

		shape->max_distance = 3.64f*Length;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = 1;
		/*
		<-1.912251*Length,-0.102928*Length> left<-------middle------->right <2.155681*Length,0.012874*Lenght>
		*/
	}
	else if (n == ground_cone_small) {
		int i;
		shape->vertex_num = 5;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = 77.5*Length / 40.0; shape->vertex[0][1] = -4459.0*Length / (219.0*40.0);
		shape->vertex[1][0] = -77.5*Length / 40.0; shape->vertex[1][1] = -4459.0*Length / (219.0*40.0);
		shape->vertex[2][0] = -77.5*Length / 40.0; shape->vertex[2][1] = 1174 * Length / (219.0*40.0);
		shape->vertex[3][0] = 0; shape->vertex[3][1] = 8243 * Length / (219.0*40.0);
		shape->vertex[4][0] = 77.5*Length / 40.0; shape->vertex[4][1] = 1174 * Length / (219.0*40.0);

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = -1;
		shape->normal[1][0] = -1; shape->normal[1][1] = 0;
		shape->normal[2][0] = -0.48516; shape->normal[2][1] = 0.87442;
		shape->normal[3][0] = 0.48516; shape->normal[3][1] = 0.87442;
		shape->normal[4][0] = 1.0; shape->normal[4][1] = 0.0;

		shape->volume = 11315.0*Length*Length / (2.0 * 40 * 40);
		shape->inertia_constant = 0.2; // assumption value
		shape->number = 98;

		shape->max_distance = 80.13*Length/40;

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = 1;
		/*
		vert[1]-----------------vert[0]
	    	|                      |
		vert[2]                 vert[4]
	           	\              /
	            	vert[3]
		*/
	}
	else if (n == ground) {
		int i;
		shape->vertex_num = 4;
		shape->vertex = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (float *)malloc(sizeof(float) * 2);
		shape->vertex[0][0] = 7 * Length; shape->vertex[0][1] = 2*Length;
		shape->vertex[1][0] = -7 * Length; shape->vertex[1][1] = 2*Length;
		shape->vertex[2][0] = -7 * Length; shape->vertex[2][1] = -2*Length;
		shape->vertex[3][0] = 7 * Length; shape->vertex[3][1] = -2*Length;

		shape->normal = (float **)malloc(sizeof(float *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (float *)malloc(sizeof(float) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = 1;
		shape->normal[1][0] = -1; shape->normal[1][1] = 0;
		shape->normal[2][0] = 0; shape->normal[2][1] = -1;
		shape->normal[3][0] = 1; shape->normal[3][1] = 0;

		shape->volume = 56 * Length*Length;
		shape->inertia_constant = 5 / 18.0;
		shape->number = 99;

		shape->max_distance = Length*sqrt(53);

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;

		shape->layer = 1;
		/*
		n[0]
		|
		vert[1]-------------------------vert[0]
		|			|			|			|
		|			|			|			|
		n[1]-|			|			|			|-n[3]
		|			|			|			|
		vert[2]-------------------------vert[3]
		|
		n[2]
		*/
	}
};

float click_perception(float mouse_x, float mouse_y, Object * A) { // find the pedetration depth of B about A: B sink to the A
	int i;
	float depth;
	float best_depth = -FLT_MAX;
	float a_vertex[2]; // vertex coordinate o f A about origin
	float b_vertex[2]; // vertex coordinate of B about origin 	if ((A->shape->max_distance + B->shape->max_distance)*(A->shape->max_distance + B->shape->max_distance) < (A->x[0] - B->x[0])*(A->x[0] - B->x[0]) + (A->x[1] - B->x[1])*(A->x[1] - B->x[1]))
	b_vertex[0] = mouse_x;
	b_vertex[1] = mouse_y;
	if (A->shape->max_distance*A->shape->max_distance>=(mouse_x-A->x[0])*(mouse_x - A->x[0])+ (mouse_y - A->x[1])*(mouse_y - A->x[1])) {
		for (i = 0; i < A->shape->vertex_num; i++) {
			a_vertex[0] = A->x[0] + A->shape->vertex[i][0];
			a_vertex[1] = A->x[1] + A->shape->vertex[i][1];
			depth = v_dot(b_vertex, A->shape->normal[i]) - v_dot(a_vertex, A->shape->normal[i]);
			if (depth > best_depth) best_depth = depth;
		}
	}
	else return 1;
	return best_depth; // positive means there is click, negative means there is click and dept is its absolute value
}

void reassign_vertex(Object * A) {
	float tempt[2];

	float rotmat[2][2];
	set_rot_mat(rotmat, &A->th);
	int i;

		switch (A->shape->number) {
		case 1: {
			float vertex[4][2];
			vertex[0][0] = Length / 2; vertex[0][1] = Length / 2;
			vertex[1][0] = -Length / 2; vertex[1][1] = Length / 2;
			vertex[2][0] = -Length / 2; vertex[2][1] = -Length / 2;
			vertex[3][0] = Length / 2; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			float normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			break;
		}
		case 2: {
			float vertex[4][2];
			vertex[0][0] = Length; vertex[0][1] = Length / 2;
			vertex[1][0] = -Length; vertex[1][1] = Length / 2;
			vertex[2][0] = -Length; vertex[2][1] = -Length / 2;
			vertex[3][0] = Length; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			float normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			break;
		}
		case 3: {
			float vertex[4][2];
			vertex[0][0] = 3 * Length / 2; vertex[0][1] = Length / 2;
			vertex[1][0] = -3 * Length / 2; vertex[1][1] = Length / 2;
			vertex[2][0] = -3 * Length / 2; vertex[2][1] = -Length / 2;
			vertex[3][0] = 3 * Length / 2; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			float normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			break;
		}
		case 4: {
			float vertex[3][2];
			vertex[0][0] = 0; vertex[0][1] = -Length / sqrt(3);
			vertex[1][0] = -Length / 2; vertex[1][1] = Length / (2 * sqrt(3));
			vertex[2][0] = Length / 2; vertex[2][1] = Length / (2 * sqrt(3));
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			float normal[3][2];
			normal[0][0] = -sqrt(3) / 2; normal[0][1] = -1.0 / 2;
			normal[1][0] = 0; normal[1][1] = 1;
			normal[2][0] = sqrt(3) / 2; normal[2][1] = -1.0 / 2;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			break;
		}
		case 5: {
			float vertex[3][2];
			vertex[0][0] = -Length / 3; vertex[0][1] = -2 * Length / 3;
			vertex[1][0] = -Length / 3; vertex[1][1] = Length / 3;
			vertex[2][0] = 2 * Length / 3; vertex[2][1] = Length / 3;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			float normal[3][2];
			normal[0][0] = -1; normal[0][1] = 0;
			normal[1][0] = 0; normal[1][1] = 1;
			normal[2][0] = sqrt(2) / 2; normal[2][1] = -sqrt(2) / 2;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			break;
		}
		case 6: {
			float vertex[4][2];
			vertex[0][0] = Length / 2; vertex[0][1] = -5 * Length / (6 * sqrt(3));
			vertex[1][0] = -Length / 2; vertex[1][1] = -5 * Length / (6 * sqrt(3));
			vertex[2][0] = -Length; vertex[2][1] = 2 * Length / (3 * sqrt(3));
			vertex[3][0] = Length; vertex[3][1] = 2 * Length / (3 * sqrt(3));
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			float normal[4][2];
			normal[0][0] = 0; normal[0][1] = -1;
			normal[1][0] = -sqrt(3) / 2.0; normal[1][1] = -1.0 / 2.0;
			normal[2][0] = 0; normal[2][1] = 1;
			normal[3][0] = sqrt(3) / 2.0; normal[3][1] = -1.0 / 2.0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			break;
		}
		case 8: {
			float vertex[6][2];
			vertex[0][0] = Length / 2.0; vertex[0][1] = -sqrt(3)*Length / 2.0;
			vertex[1][0] = -Length / 2.0; vertex[1][1] = -sqrt(3)*Length / 2.0;
			vertex[2][0] = -Length; vertex[2][1] = 0;
			vertex[3][0] = -Length / 2.0; vertex[3][1] = sqrt(3)*Length / 2.0;
			vertex[4][0] = Length / 2.0; vertex[4][1] = sqrt(3)*Length / 2.0;
			vertex[5][0] = Length; vertex[5][1] = 0;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			mat_v_product(rotmat, vertex[5], A->shape->vertex[5]);
			float normal[6][2];
			normal[0][0] = 0; normal[0][1] = -1;
			normal[1][0] = -sqrt(3) / 2.0; normal[1][1] = -1.0 / 2.0;
			normal[2][0] = -sqrt(3) / 2.0; normal[2][1] = 1.0 / 2.0;
			normal[3][0] = 0; normal[3][1] = 1;
			normal[4][0] = sqrt(3) / 2.0; normal[4][1] = 1.0 / 2.0;
			normal[5][0] = sqrt(3) / 2.0; normal[5][1] = -1.0 / 2.0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			mat_v_product(rotmat, normal[5], A->shape->normal[5]);
			break;
		}
		case 9: {
			float vertex[5][2];
			vertex[0][0] = 2.0*Length / 21.0; vertex[0][1] = -23.0*Length / 21.0;
			vertex[1][0] = -19.0*Length / 21.0; vertex[1][1] = -23.0*Length / 21.0;
			vertex[2][0] = -19.0*Length / 21.0; vertex[2][1] = 19.0*Length / 21.0;
			vertex[3][0] = 23.0*Length / 21.0; vertex[3][1] = 19.0*Length / 21.0;
			vertex[4][0] = 23.0*Length / 21.0; vertex[4][1] = -2.0*Length / 21.0;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			float normal[5][2];
			normal[0][0] = 0; normal[0][1] = -1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = 1;
			normal[3][0] = 1; normal[3][1] = 0;
			normal[4][0] = sqrt(2) / 2.0; normal[4][1] = -sqrt(2) / 2.0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			break;
		}
		case 80: {
			float vertex[11][2];
			vertex[0][0] = 12.036667*Length / 40.0; vertex[0][1] = -33.022701*Length / 40.0;
			vertex[1][0] = -29.963333*Length / 40.0; vertex[1][1] = -26.022701*Length / 40.0;
			vertex[2][0] = -48.963333*Length / 40.0; vertex[2][1] = 6.977299*Length / 40.0;
			vertex[3][0] = -49.963333*Length / 40.0; vertex[3][1] = 14.977299*Length / 40.0;
			vertex[4][0] = -41.963333*Length / 40.0; vertex[4][1] = 26.977299*Length / 40.0;
			vertex[5][0] = -22.963333*Length / 40.0; vertex[5][1] = 34.977299*Length / 40.0;
			vertex[6][0] = 34.036667*Length / 40.0; vertex[6][1] = 20.977299*Length / 40.0;
			vertex[7][0] = 42.036667*Length / 40.0; vertex[7][1] = 12.977299*Length / 40.0;
			vertex[8][0] = 49.036667*Length / 40.0; vertex[8][1] = -1.022701*Length / 40.0;
			vertex[9][0] = 49.036667*Length / 40.0; vertex[9][1] = -15.022701*Length / 40.0;
			vertex[10][0] = 41.036667*Length / 40.0; vertex[10][1] = -27.022701*Length / 40.0;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			mat_v_product(rotmat, vertex[5], A->shape->vertex[5]);
			mat_v_product(rotmat, vertex[6], A->shape->vertex[6]);
			mat_v_product(rotmat, vertex[7], A->shape->vertex[7]);
			mat_v_product(rotmat, vertex[8], A->shape->vertex[8]);
			mat_v_product(rotmat, vertex[9], A->shape->vertex[9]);
			mat_v_product(rotmat, vertex[10], A->shape->vertex[10]);
			float normal[11][2];
			normal[0][0] = -0.164399; normal[0][1] = -0.986394;
			normal[1][0] = -0.866622; normal[1][1] = -0.498964;
			normal[2][0] = -0.992278; normal[2][1] = -0.124035;
			normal[3][0] = -0.832050; normal[3][1] = 0.554700;
			normal[4][0] = -0.388057; normal[4][1] = 0.921635;
			normal[5][0] = 0.238525; normal[5][1] = 0.971136;
			normal[6][0] = 0.707107; normal[6][1] = 0.707107;
			normal[7][0] = 0.894427; normal[7][1] = 0.447214;
			normal[8][0] = 1.000000; normal[8][1] = 0.000000;
			normal[9][0] = 0.832050; normal[9][1] = -0.554700;
			normal[10][0] = 0.202606; normal[10][1] = -0.979260;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			mat_v_product(rotmat, normal[5], A->shape->normal[5]);
			mat_v_product(rotmat, normal[6], A->shape->normal[6]);
			mat_v_product(rotmat, normal[7], A->shape->normal[7]);
			mat_v_product(rotmat, normal[8], A->shape->normal[8]);
			mat_v_product(rotmat, normal[9], A->shape->normal[9]);
			mat_v_product(rotmat, normal[10], A->shape->normal[10]);
			break;
		}
		case 81: {
			float vertex[6][2];
			vertex[0][0] = -0.558511*Length; vertex[0][1] = -0.759908*Length;
			vertex[1][0] = -0.408511*Length; vertex[1][1] = 0.765092*Length;
			vertex[2][0] = 0.491489*Length; vertex[2][1] = 0.765092*Length;
			vertex[3][0] = 0.516489*Length; vertex[3][1] = -0.509908*Length;
			vertex[4][0] = 0.466489*Length; vertex[4][1] = -0.609908*Length;
			vertex[5][0] = 0.016489*Length; vertex[5][1] = -0.759908*Length;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			mat_v_product(rotmat, vertex[5], A->shape->vertex[5]);
			float normal[6][2];
			normal[0][0] = -0.995197; normal[0][1] = 0.097888;
			normal[1][0] = -0.000000; normal[1][1] = 1.000000;
			normal[2][0] = 0.999808; normal[2][1] = 0.019604;
			normal[3][0] = 0.894427; normal[3][1] = -0.447214;
			normal[4][0] = 0.316228; normal[4][1] = -0.948683;
			normal[5][0] = -0.000000; normal[5][1] = -1.000000;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			mat_v_product(rotmat, normal[5], A->shape->normal[5]);
			break;
		}
		case 95: {
			Object * tempt_pointer; tempt_pointer = A + 1;
			set_rot_mat(rotmat, &tempt_pointer->th);
			float vertex[5][2];
			vertex[0][0] = -1.007442*Length; vertex[0][1] = -2.040490*Length;
			vertex[1][0] = -0.157442*Length; vertex[1][1] = 3.309510*Length;
			vertex[2][0] = 0.617558*Length; vertex[2][1] = 3.384510*Length;
			vertex[3][0] = 0.617558*Length; vertex[3][1] = -2.865490*Length;
			vertex[4][0] = 0.217558*Length; vertex[4][1] = -2.940490*Length;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			float normal[5][2];
			normal[0][0] = -0.987613; normal[0][1] = 0.156910;
			normal[1][0] = -0.096324; normal[1][1] = 0.995350;
			normal[2][0] = 1.000000; normal[2][1] = 0.000000;
			normal[3][0] = 0.184289; normal[3][1] = -0.982872;
			normal[4][0] = -0.592076; normal[4][1] = -0.805882;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
/*
float rotmat[2][2];
float radius = angle*pi / 180;
float coordinate[2];
float output[2];
set_rot_mat(rotmat, &radius);

set_object(&object[population - 2], 97, 99, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);

coordinate[0] = -1.912251*Length*trans; coordinate[1] = -0.102928*Length*trans;
mat_v_product(rotmat, coordinate, output);
set_object(&object[population - 3], 95, 99, mouse_x + output[0], mouse_y + output[1], 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);

coordinate[0] = 2.155681*Length*trans; coordinate[1] = 0.012874*Length*trans;
mat_v_product(rotmat, coordinate, output);
set_object(&object[population - 1], 96, 99, mouse_x + output[0], mouse_y + output[1], 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
/*
<-1.912251*Length,-0.102928*Length> left<-------middle------->right <2.155681*Length, 0.012874*Lenght>
*/
			float coordinate[2]; coordinate[0] = -1.912251*Length; coordinate[1] = -0.102928*Length;
			float output[2];
			mat_v_product(rotmat, coordinate, output);
			A->x[0] = tempt_pointer->x[0] + output[0];
			A->x[1] = tempt_pointer->x[1] + output[1];
			break;
		}
		case 96: {
			Object * tempt_pointer; tempt_pointer = A - 1;
			set_rot_mat(rotmat, &tempt_pointer->th);
			float vertex[6][2];
			vertex[0][0] = -0.877352*Length; vertex[0][1] = -3.056292*Length;
			vertex[1][0] = -0.877352*Length; vertex[1][1] = 3.393708*Length;
			vertex[2][0] = 0.647648*Length; vertex[2][1] = 3.168708*Length;
			vertex[3][0] = 1.122648*Length; vertex[3][1] = -2.356292*Length;
			vertex[4][0] = 0.872648*Length; vertex[4][1] = -2.806292*Length;
			vertex[5][0] = -0.252352*Length; vertex[5][1] = -3.156292*Length;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			mat_v_product(rotmat, vertex[5], A->shape->vertex[5]);
			float normal[6][2];
			normal[0][0] = -1.000000; normal[0][1] = 0.000000;
			normal[1][0] = 0.145961; normal[1][1] = 0.989290;
			normal[2][0] = 0.996325; normal[2][1] = 0.085657;
			normal[3][0] = 0.874157; normal[3][1] = -0.485643;
			normal[4][0] = 0.297067; normal[4][1] = -0.954857;
			normal[5][0] = -0.157991; normal[5][1] = -0.987441;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			mat_v_product(rotmat, normal[5], A->shape->normal[5]);
			/*
			float rotmat[2][2];
			float radius = angle*pi / 180;
			float coordinate[2];
			float output[2];
			set_rot_mat(rotmat, &radius);

			set_object(&object[population - 2], 97, 99, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);

			coordinate[0] = -1.912251*Length*trans; coordinate[1] = -0.102928*Length*trans;
			mat_v_product(rotmat, coordinate, output);
			set_object(&object[population - 3], 95, 99, mouse_x + output[0], mouse_y + output[1], 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);

			coordinate[0] = 2.155681*Length*trans; coordinate[1] = 0.012874*Length*trans;
			mat_v_product(rotmat, coordinate, output);
			set_object(&object[population - 1], 96, 99, mouse_x + output[0], mouse_y + output[1], 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
			/*
			<-1.912251*Length,-0.102928*Length> left<-------middle------->right <2.155681*Length, 0.012874*Lenght>
			*/
			float coordinate[2]; coordinate[0] = 2.155681*Length; coordinate[1] = 0.012874*Length;
			float output[2];
			mat_v_product(rotmat, coordinate, output);
			A->x[0] = tempt_pointer->x[0] + output[0];
			A->x[1] = tempt_pointer->x[1] + output[1];
			break;
		}
		case 97: {
			float vertex[5][2];
			vertex[0][0] = -1.296671*Length; vertex[0][1] = -2.968418*Length;
			vertex[1][0] = -1.296671*Length; vertex[1][1] = 3.281582*Length;
			vertex[2][0] = 1.278329*Length; vertex[2][1] = 3.406582*Length;
			vertex[3][0] = 1.278329*Length; vertex[3][1] = -3.243418*Length;
			vertex[4][0] = -0.321671*Length; vertex[4][1] = -3.543418*Length;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			float normal[5][2];
			normal[0][0] = -1.000000; normal[0][1] = 0.000000;
			normal[1][0] = -0.048487; normal[1][1] = 0.998824;
			normal[2][0] = 1.000000; normal[2][1] = 0.000000;
			normal[3][0] = 0.184289; normal[3][1] = -0.982872;
			normal[4][0] = -0.507985; normal[4][1] = -0.861366;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			break;
		}
		case 98: {
			float vertex[5][2];
			vertex[0][0] = 77.5*Length / 40.0; vertex[0][1] = -4459.0*Length / (219.0*40.0);
			vertex[1][0] = -77.5*Length / 40.0; vertex[1][1] = -4459.0*Length / (219.0*40.0);
			vertex[2][0] = -77.5*Length / 40.0; vertex[2][1] = 1174.0 * Length / (219.0*40.0);
			vertex[3][0] = 0.0; vertex[3][1] = 8243.0 * Length / (219.0*40.0);
			vertex[4][0] = 77.5*Length / 40.0; vertex[4][1] = 1174.0 * Length / (219.0*40.0);
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			mat_v_product(rotmat, vertex[4], A->shape->vertex[4]);
			float normal[5][2];
			normal[0][0] = 0.0; normal[0][1] = -1.0;
			normal[1][0] = -1.0; normal[1][1] = 0.0;
			normal[2][0] = -0.48516; normal[2][1] = 0.87442;
			normal[3][0] = 0.48516; normal[3][1] = 0.87442;
			normal[4][0] = 1.0; normal[4][1] = 0.0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			mat_v_product(rotmat, normal[4], A->shape->normal[4]);
			break;
		}
		case 99: {
			float vertex[4][2];
			vertex[0][0] = 7 * Length; vertex[0][1] = 2 * Length;
			vertex[1][0] = -7 * Length; vertex[1][1] = 2 * Length;
			vertex[2][0] = -7 * Length; vertex[2][1] = -2 * Length;
			vertex[3][0] = 7 * Length; vertex[3][1] = -2 * Length;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			float normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
			break;
		}
		}
}

void delta_zero(Object * A) {
	A->delv[0] = 0;
	A->delv[1] = 0;
	A->delw = 0;
}

void stability_zero(Object * A) {
	int i;
	if (A->material.number != 99) A->stable = false;
	else A->stable= true;
}

void collision_check(Object * A, Object * B, int NumberOfObject, cinf contact[], int numA, int numB) { // Let B sink to the A
	float tempt_vector[2]; // temporary vector for calculus
	float rA[2]; // position vector of collision point from COM of A
	float rB[2]; // position vector of collision point from COM of B
	float VBA[2]; // VB-VA
	float tangent[2];
	int indexA, indexB;
	int i;
	int index;
	float depth;
	float drA[2], drB[2];

	for (indexB = 0; indexB < B->shape->vertex_num; indexB++) {
		depth = find_penetration_depth(A, B, &indexB, &indexA);
		if (depth <= 0) {
			float VA[2], VB[2];
			float e = minimum(A->material.restitution, B->material.restitution);

			B->shape->vertex_contact[indexB] = true;
			rB[0] = B->shape->vertex[indexB][0]; rB[1] = B->shape->vertex[indexB][1]; // set rB
			v_sum(B->x, rB, tempt_vector); v_sub(tempt_vector, A->x, rA); // set rA
			v_cross_k_inv(A->w, rA, tempt_vector); v_sum(tempt_vector, A->v, VA); // set VA
			VA[0] = VA[0];	VA[1] = VA[1];
			v_cross_k_inv(B->w, rB, tempt_vector); v_sum(tempt_vector, B->v, VB); // set VB
			VB[0] = VB[0];	VB[1] = VB[1];
			v_sub(VB, VA, VBA); // set VBA

			if (B->shape->vertex_contact_index[indexB] == -1) {
				for (i = 0; i < 4 * NumberOfObject; i++) if (contact[i].run == false) {
					index = i;
					contact[index].run = true;
					contact[index].numA = numA;
					contact[index].numB = numB;
					contact[index].indexA = indexA;
					contact[index].indexB = indexB;
					contact[index].rA[0] = rA[0]; contact[index].rA[1] = rA[1];
					contact[index].rB[0] = rB[0]; contact[index].rB[1] = rB[1];
					contact[index].depth = depth;
					contact[index].normal_mass = (A->inv_mass + B->inv_mass + A->inv_inertia*v_cross(rA, A->shape->normal[indexA])*v_cross(rA, A->shape->normal[indexA]) + B->inv_inertia*v_cross(rB, A->shape->normal[indexA])*v_cross(rB, A->shape->normal[indexA]));
					v_cross_k(A->shape->normal[indexA], 1.0f, tangent);
					contact[index].tangent_mass = (A->inv_mass + B->inv_mass + A->inv_inertia*v_cross(rA, tangent)*v_cross(rA, tangent) + B->inv_inertia*v_cross(rB, tangent)*v_cross(rB, tangent));
					B->shape->vertex_contact_index[indexB] = index;
					contact[index].velocity_bias = -e*v_dot(VBA, A->shape->normal[indexA]);
					break;
				}
			}
			else {
				index = B->shape->vertex_contact_index[indexB];
				drA[0] = rA[0] - contact[index].rA[0]; drA[1] = rA[1] - contact[index].rA[1];
				drB[0] = rB[0] - contact[index].rB[0]; drB[1] = rB[1] - contact[index].rB[1];
				if (contact[index].numA == numA && contact[index].indexA == indexA && drA[0] * drA[0] + drA[1] * drA[1] < warmth&& drB[0] * drB[0] + drB[1] * drB[1] < warmth) {
					contact[index].run = true;
					contact[index].numA = numA;
					contact[index].numB = numB;
					contact[index].indexA = indexA;
					contact[index].indexB = indexB;
					contact[index].velocity_bias = -e*v_dot(VBA, A->shape->normal[indexA]);
				}
				else {
					contact[index].run = true;
					contact[index].numA = numA;
					contact[index].numB = numB;
					contact[index].indexA = indexA;
					contact[index].indexB = indexB;
					contact[index].rA[0] = rA[0]; contact[index].rA[1] = rA[1];
					contact[index].rB[0] = rB[0]; contact[index].rB[1] = rB[1];
					contact[index].depth = depth;
					contact[index].normal_mass = (A->inv_mass + B->inv_mass + A->inv_inertia*v_cross(rA, A->shape->normal[indexA])*v_cross(rA, A->shape->normal[indexA]) + B->inv_inertia*v_cross(rB, A->shape->normal[indexA])*v_cross(rB, A->shape->normal[indexA]));
					v_cross_k(A->shape->normal[indexA], 1.0f, tangent);
					contact[index].tangent_mass = (A->inv_mass + B->inv_mass + A->inv_inertia*v_cross(rA, tangent)*v_cross(rA, tangent) + B->inv_inertia*v_cross(rB, tangent)*v_cross(rB, tangent));
					contact[index].normal_impulse = 0;
					contact[index].tangent_impulse = 0;
					contact[index].velocity_bias = -e*v_dot(VBA, A->shape->normal[indexA]);
				}
			}
		}
	}
}

void uncollide_check(Object A[], int NumberOfObject, cinf contact[]) {
	int i, j;
	int index;
	for (i = 0; i < NumberOfObject; i++) {
		for (j = 0; j < A[i].shape->vertex_num;j++) {
			if (A[i].shape->vertex_contact[j] == false && A[i].shape->vertex_contact_index[j] != -1) {
				index = A[i].shape->vertex_contact_index[j];
				contact[index].run = false;
				contact[index].numA = 0;
				contact[index].numB = 0;
				contact[index].indexA = 0;
				contact[index].indexB = 0;
				contact[index].rA[0] = 0;
				contact[index].rA[1] = 0;
				contact[index].rB[0] = 0;
				contact[index].rB[1] = 0;
				contact[index].depth = 0;
				contact[index].normal_impulse = 0;
				contact[index].tangent_impulse = 0;
				contact[index].normal_mass = 0;
				contact[index].tangent_mass = 0;
				contact[index].velocity_bias = 0;
				A[i].shape->vertex_contact_index[j] = -1;
			}
		}
	}
	for (i = 0; i < NumberOfObject; i++) {
		for (j = 0; j < A[i].shape->vertex_num; j++) {
			A[i].shape->vertex_contact[j] = false;
		}
	}
}

void warmstarting(Object A[], int NumberOfObject, cinf contact[]) {
	int i, j;
	int index, indexA;
	int numA, numB;
	float dt = 1 / fpss;
	float tangent[2];

	for (i = 0; i < NumberOfObject; i++) {
		for (j = 0; j < A[i].shape->vertex_num; j++) {
			if (A[i].shape->vertex_contact_index[j] != -1) {
				index = A[i].shape->vertex_contact_index[j];
				numA = contact[index].numA;
				numB = contact[index].numB;
				indexA = contact[index].indexA;
				v_cross_k(A[numA].shape->normal[indexA], 1.0f, tangent);
				A[numB].v[0] = A[numB].v[0] + contact[index].normal_impulse * A[numB].inv_mass*A[numA].shape->normal[indexA][0];
				A[numB].v[1] = A[numB].v[1] + contact[index].normal_impulse * A[numB].inv_mass*A[numA].shape->normal[indexA][1];
				A[numA].v[0] = A[numA].v[0] - contact[index].normal_impulse * A[numA].inv_mass*A[numA].shape->normal[indexA][0];
				A[numA].v[1] = A[numA].v[1] - contact[index].normal_impulse * A[numA].inv_mass*A[numA].shape->normal[indexA][1];
				A[numB].w = A[numB].w + contact[index].normal_impulse*v_cross(contact[index].rB, A[numA].shape->normal[indexA])* A[numB].inv_inertia;
				A[numA].w = A[numA].w - contact[index].normal_impulse*v_cross(contact[index].rA, A[numA].shape->normal[indexA])* A[numA].inv_inertia;

				A[numB].v[0] = A[numB].v[0] + contact[index].tangent_impulse * A[numB].inv_mass*tangent[0];
				A[numB].v[1] = A[numB].v[1] + contact[index].tangent_impulse * A[numB].inv_mass*tangent[1];
				A[numA].v[0] = A[numA].v[0] - contact[index].tangent_impulse * A[numA].inv_mass*tangent[0];
				A[numA].v[1] = A[numA].v[1] - contact[index].tangent_impulse * A[numA].inv_mass*tangent[1];
				A[numB].w = A[numB].w + contact[index].tangent_impulse*v_cross(contact[index].rB, tangent)* A[numB].inv_inertia;
				A[numA].w = A[numA].w - contact[index].tangent_impulse*v_cross(contact[index].rA, tangent)* A[numA].inv_inertia;
			}
		}
	}
}

void one_term_with_earthquake(Object A[], int NumberOfObject, cinf contact[], int earthquake_type, float *time, float amp, float frequency) {
	int i, j, k;

	for (i = 0; i < NumberOfObject; i++) {
		A[i].delv[0] = 0;
		A[i].delv[1] = 0;
		A[i].delw = 0;
	}

	for (i = 0; i < NumberOfObject; i++) { force_zero(&A[i]); gravity(&A[i]); }
	for (i = 0; i < NumberOfObject; i++) if (A[i].material.number == 99) earthquake(&A[i], *time, earthquake_type, amp, frequency);
	for (i = 0; i < NumberOfObject; i++) { apply_force(&A[i]); }

	for (i = 0; i < NumberOfObject; i++) {
		for (j = 0; j < NumberOfObject; j++) {
			if (i > j) if (!(A[i].shape->layer>0 && A[j].shape->layer>0 && (A[i].shape->layer == A[j].shape->layer))) { collision_check(&A[j], &A[i], NumberOfObject, contact, j, i); collision_check(&A[i], &A[j], NumberOfObject, contact, i, j); }
		}
	}

	uncollide_check(A, NumberOfObject, contact);

	warmstarting(A, NumberOfObject, contact);

	for (j = 0; j < 10; j++) {
		for (i = 0; i < NumberOfObject; i++) {
			cycle1(A, i, NumberOfObject, contact);
		}
	}

	for (i = 0; i < NumberOfObject; i++) { movement(&A[i]); }
	for (k = 0; k < 3;k++) for (i = 0; i < NumberOfObject; i++) for (j = 0; j < NumberOfObject; j++) if (i > j) if (!(A[i].shape->layer>0 && A[j].shape->layer>0 && (A[i].shape->layer == A[j].shape->layer))) { positionalcorrection(&A[i], &A[j]); positionalcorrection(&A[j], &A[i]); }
	*time = *time + 1.0 / fpss;

	for (i = 0; i < NumberOfObject; i++) if (A[i].material.number == 99) { A[i].v[0] = 0; A[i].v[1] = 0; A[i].w = 0; }
}

void earthquake(Object * A, float time, int type, float amp, float frequency) {
	enum ShakeFuntion { stay = 0, cosine = 1, sine, updowncosine, updownsine, cosine_updonwcosine, rotationcosine, circle, swing };

	switch (type) {
	case cosine: A->v[0] = amp*cos(time * 2 * pi*frequency); break;
	case sine: A->v[0] = amp*sin(time * 2 * pi*frequency); break;
	case updowncosine: A->v[1] = amp*cos(time * 2 * pi*frequency); break;
	case updownsine: A->v[1] = amp*sin(time * 2 * pi*frequency); break;
	case cosine_updonwcosine: { A->v[0] = amp*cos(time * 2 * pi*frequency); A->v[1] = amp*cos(time * 2 * pi*frequency); break; }
	case rotationcosine: {
		A->w = amp*cos(time * 2 * pi*frequency);
		if (A->shape->number == 95) {
			Object * tempt_A; tempt_A = A + 1;
			float position[2]; position[0] = -1.912251f*Length; position[1] = -0.102928f*Length;
			float revise_position[2];
			float rotmat[2][2];
			set_rot_mat(rotmat, &A->th);
			mat_v_product(rotmat, position, revise_position);
			float v[2];
			v_cross_k_inv(A->w, revise_position, v);
			A->v[0] = A->v[0] + v[0]; A->v[1] = A->v[1] + v[1];
			A->th = tempt_A->th;
			A->x[0] = tempt_A->x[0] + revise_position[0];
			A->x[1] = tempt_A->x[1] + revise_position[1];
		}
		else if (A->shape->number == 96) {
			Object * tempt_A; tempt_A = A - 1;
			float position[2]; position[0] = 2.155681f*Length; position[1] = 0.012874f*Length;
			float revise_position[2];
			float rotmat[2][2];
			set_rot_mat(rotmat, &A->th);
			mat_v_product(rotmat, position, revise_position);
			float v[2];
			v_cross_k_inv(A->w, revise_position, v);
			A->v[0] = A->v[0] + v[0]; A->v[1] = A->v[1] + v[1];
			A->th = tempt_A->th;
			A->x[0] = tempt_A->x[0] + revise_position[0];
			A->x[1] = tempt_A->x[1] + revise_position[1];
		}
		break;
	}
	case circle: { A->v[0] = -amp * 2 * pi*frequency*cos(2 * pi*frequency*time); A->v[1] = -amp * 2 * pi*frequency*sin(2 * pi*frequency*time); break; }
	case swing: {
		A->v[0] = -amp*(pi / 4.0)*(2 * pi*frequency)*cos(2 * pi*frequency*time)*cos((pi / 4.0)*sin(2 * pi*frequency*time));
		A->v[1] = -amp*(pi / 4.0)*(2 * pi*frequency)*cos(2 * pi*frequency*time)*sin((pi / 4.0)*sin(2 * pi*frequency*time));
		A->w = (2 * pi*frequency)*(pi / 4.0)*cos(2 * pi*frequency*time);
		if (A->shape->number == 95) {
			Object * tempt_A; tempt_A = A + 1;
			float position[2]; position[0] = -1.912251f*Length; position[1] = -0.102928f*Length;
			float revise_position[2];
			float rotmat[2][2];
			set_rot_mat(rotmat, &A->th);
			mat_v_product(rotmat, position, revise_position);
			float v[2];
			v_cross_k_inv(A->w, revise_position, v);
			A->v[0] = A->v[0] + v[0]; A->v[1] = A->v[1] + v[1];
			A->th = tempt_A->th;
			A->x[0] = tempt_A->x[0] + revise_position[0];
			A->x[1] = tempt_A->x[1] + revise_position[1];
		}
		else if (A->shape->number == 96) {
			Object * tempt_A; tempt_A = A - 1;
			float position[2]; position[0] = 2.155681f*Length; position[1] = 0.012874f*Length;
			float revise_position[2];
			float rotmat[2][2];
			set_rot_mat(rotmat, &A->th);
			mat_v_product(rotmat, position, revise_position);
			float v[2];
			v_cross_k_inv(A->w, revise_position, v);
			A->v[0] = A->v[0] + v[0]; A->v[1] = A->v[1] + v[1];
			A->th = tempt_A->th;
			A->x[0] = tempt_A->x[0] + revise_position[0];
			A->x[1] = tempt_A->x[1] + revise_position[1];
		}
		break;
	}
	}
}
/*
<-1.912251*Length,-0.102928*Length> left<-------middle------->right <2.155681*Length,0.012874*Lenght>
*/

void map_editor(Object **A, int * NumberOfObject,int * budget, float * height,float * starttime, float * timeduration, float * endtime, int * earthquake_type, float *amp, float *frequency, int stage, int step, bool * explo, int * bomb_population, Bomb ** bomb) {
	Object * function_tempt;
	Bomb * function_bomb_tempt;
	*explo = true;
	*bomb_population = 0;
	int i;
	if (stage == 1) {
		if (step == 1) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 20; // previous : 36
			*height = 500;
			*starttime = 1;
			*timeduration = 8;
			*endtime = 9;
			*earthquake_type = 1;
			*amp = 0.5;
			*frequency = 0.5;
		}
		else if (step == 2) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 30; // previous : 56
			*height = 420;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 3;
			*amp = 1;
			*frequency = 0.5;
		}
		else if (step == 3) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 70; // previous : 115
			*height = 300;
			*starttime = 1;
			*timeduration = 6.5;
			*endtime = 7.5;
			*earthquake_type = 5;
			*amp = 1;
			*frequency = 1.0;
		}
		else if (step == 4) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 70; // minimum : 68, previous : 250
			*height = 200;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 6;
			*amp = 1.5;
			*frequency = 2.5;
		}
		else if (step == 5) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, pi/18.0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 45; // previous : 440
			*height = 280;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 1;
			*amp = 1.5;
			*frequency = 1.5;
		}
		else if (step == 6) {
			*NumberOfObject = 2;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 98, 99, 406, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 98, 99, 616, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 285; // minimum : 282, previous : 500
			*height = 250;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 5;
			*amp = 1.5;
			*frequency = 2.0;
		}
		else if (step == 7) {
			*NumberOfObject = 2;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 98, 99, 370, 707, 0, 0, 0, 0, -pi / 18.0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 98, 99, 590, 707, 0, 0, 0, 0, pi / 18.0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 70;
			*height = 250;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 6;
			*amp = 0.8;
			*frequency = 2.0;
		}
		else if (step == 8) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 98, 99, 480, 707, 0, 0, 0, 0,0.0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 60; // minimum : 60, previous : 100
			*height = 210;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 6;
			*amp = 0.2;
			*frequency = 0.05;
		}
	}
	else if (stage == 2) {
		if (step == 1) {
			*NumberOfObject = 2;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 80, 98, 200, 287, 10*trans, -3*trans, 0, 0, 0.1, 1, 0, 0, 0);
			*A = function_tempt;
			*budget = 50; // minimum : 50, preciouse : 70
			*height = 270;
			*starttime = 1;
			*timeduration = 3;
			*endtime = 4;
			*earthquake_type = 0;
			*amp = 0.2;
			*frequency = 0.05;
		}
		else if (step == 2) {
			*NumberOfObject = 3;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 80, 98, 200, 287, 10 * trans, -3 * trans, 0, 0, 0.1, 1, 0, 0, 0);
			set_object(&function_tempt[2], 80, 98, 800, 307, -15 * trans, 2 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			*A = function_tempt;
			*budget = 105;
			*height = 270;
			*starttime = 1;
			*timeduration = 5;
			*endtime = 6;
			*earthquake_type = 5;
			*amp = 1.0;
			*frequency = 1.0;
		}
		else if (step == 3) {
			*NumberOfObject = 5;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 80, 98, 100, 50, 1 * trans, 3 * trans, 0, 0, 0.1, 1, 0, 0, 0);
			set_object(&function_tempt[2], 80, 98, 300, 50, 1 * trans, 3 * trans, 0, 0, 0.1, 1, 0, 0, 0);
			set_object(&function_tempt[3], 80, 98, 500, 50, 1 * trans, 3 * trans, 0, 0, 0.1, 1, 0, 0, 0);
			set_object(&function_tempt[4], 80, 98, 700, 50, 1 * trans, 3 * trans, 0, 0, 0.1, 1, 0, 0, 0);
			*A = function_tempt;
			*budget = 150;
			*height = 270;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 5;
			*amp = 0.0;
			*frequency = 1.0;
		}
		else if (step == 4) {
			*NumberOfObject = 2;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 98, 99, 480, 707, 0, 0, 0, 0, 0.0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 80, 98, 300, 50, 1 * trans, 3 * trans, 0, 0, 0.1, 1, 0, 0, 0);
			*A = function_tempt;
			*budget = 100;
			*height = 210;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 6;
			*amp = 0.2;
			*frequency = 0.05;
		}
		else if (step == 5) {
			*NumberOfObject = 2;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 98, 99, 480, 707, 0, 0, 0, 0, 0.0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 80, 98, 250, 50, 0, 15 * trans, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 60;
			*height = 190;
			*starttime = 1;
			*timeduration = 3;
			*endtime = 4;
			*earthquake_type = 0;
			*amp = 0.2;
			*frequency = 0.05;
		}
		else if (step == 6) {
			*NumberOfObject = 3;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[1], 97, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 96, 99, 536 + 86.22724, 727 + 0.51496, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[0], 95, 99, 536- 76.56916, 727 - 3.5, 0, 0, 0, 0, 0, 0, 0, 0, 0); // 0.411712
			*A = function_tempt;
			*budget = 270;
			*height = 250;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 1;
			*amp = 3;
			*frequency = 1.0;
		}
		else if (step == 7) {
			*NumberOfObject = 6;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[1], 97, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 96, 99, 536 + 86.22724, 727 + 0.51496, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[0], 95, 99, 536 - 76.56916, 727 - 3.5, 0, 0, 0, 0, 0, 0, 0, 0, 0); // 0.411712
			set_object(&function_tempt[3], 80, 98, 236, 287, 15 * trans, -2 * trans, 0, 0, 0.1, 1, 0, 0, 0);
			set_object(&function_tempt[4], 80, 98, 836, 307, -15 * trans, 2 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[5], 80, 98, 236, 457, 15 * trans, -2 * trans, 0, 0, 0.5, 1, 0, 0, 0);
			*A = function_tempt;
			*budget = 210;
			*height = 300;
			*starttime = 1;
			*timeduration = 5;
			*endtime = 6;
			*earthquake_type = 0;
			*amp = 3;
			*frequency = 1.0;
		}
		else if (step == 8) {
			*NumberOfObject = 5;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 80, 98, 306, 600, 0, -15 * trans, 0, 0, 0.5, 1, 0, 0, 0);
			set_object(&function_tempt[2], 80, 98, 456, 600, 0, -15 * trans, 0, 0, 0.5, 1, 0, 0, 0);
			set_object(&function_tempt[3], 80, 98, 606, 600, 0, -15 * trans, 0, 0, 0.5, 1, 0, 0, 0);
			set_object(&function_tempt[4], 80, 98, 756, 600, 0, -15 * trans, 0, 0, 0.5, 1, 0, 0, 0);
			*A = function_tempt;
			*budget = 150;
			*height = 300;
			*starttime = 1;
			*timeduration = 5;
			*endtime = 6;
			*earthquake_type = 6;
			*amp = -0.2;
			*frequency = 0.05;
		}
	}
	else if (stage == 3) {
		if (step == 1) {
			*NumberOfObject = 5;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 1, 6, 536, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 586, 580, 0, 0, 0, 0, pi/2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 486, 580, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 81, 97, 536, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 15;
			*height = 540;
			*starttime = 1;
			*timeduration = 3;
			*endtime = 4;
			*earthquake_type = 0;
			*amp = 0;
			*frequency = 0.05;
		}
		else if (step == 2) {
			*NumberOfObject = 10;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 536, 500, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 586, 580, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 486, 580, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 81, 97, 536, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 5, 536, 340, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 3, 5, 586, 420, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[7], 3, 5, 486, 420, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[8], 81, 97, 536, 450, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[9], 80, 98, 856, 337, -15 * trans, 2 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			*A = function_tempt;
			*budget = 45;
			*height = 470;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 0;
			*amp = 0;
			*frequency = 0.05;
		}
		else if (step == 3) {
			*NumberOfObject = 4;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 81, 97, 506, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 81, 97, 566, 390, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[3], 80, 98, 200, 50, 10 * trans, 10 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			*A = function_tempt;
			*budget = 100;
			*height = 350;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 1;
			*amp = 2.0;
			*frequency = 2.0;
		}
		else if (step == 4) {
			*NumberOfObject = 7;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 81, 97, 506, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 81, 97, 606, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[3], 81, 97, 406, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[4], 81, 97, 406, 480, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[5], 81, 97, 506, 480, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 80, 98, 550, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 250;
			*height = 300;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 3;
			*amp = 1.5;
			*frequency = 2.0;
		}
		else if (step == 5) {
			*NumberOfObject = 6;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 98, 99, 536, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 98, 99, 336, 600, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 81, 97, 336, 540, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[3], 80, 98, 100, 50, 1 * trans, 3 * trans, 0, 0, 0.1, 3.0, 0, 0, 0);
			set_object(&function_tempt[4], 80, 98, 300, 50, 1 * trans, 3 * trans, 0, 0, 0.1, 3.0, 0, 0, 0);
			set_object(&function_tempt[5], 80, 98, 500, 50, 1 * trans, 3 * trans, 0, 0, 0.1, 3.0, 0, 0, 0);
			*A = function_tempt;
			*budget = 70;
			*height = 250;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 1;
			*amp = 2.0;
			*frequency = 1.0;
		}
		else if (step == 6) {
			*NumberOfObject = 10;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 81, 97, 350, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 81, 97, 450, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[3], 81, 97, 550, 450, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[4], 81, 97, 650, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[5], 81, 97, 750, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 3, 5, 520, 580, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[7], 3, 5, 580, 580, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[8], 3, 5, 550, 500, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[9], 80, 98, 200, 150, 20 * trans, 0, 0, 0, 0.5, -1, 0, 0, 0);
			*A = function_tempt;
			*budget = 100;
			*height = 100;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 6;
			*amp = 0.4;
			*frequency = 1.0;
		}
		else if (step == 7) {
			*NumberOfObject = 10;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 6, 650, 580, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 6, 650, 460, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 6, 650, 340, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 6, 610, 580, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 6, 610, 460, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[6], 3, 6, 610, 340, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[7], 80, 98, 900, 450, -15*trans, 0, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[8], 81, 97, 350, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[9], 81, 97, 450, 610, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 60;
			*height = 500;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 6;
			*amp = -0.1;
			*frequency = 0.05;
		}
		else if (step == 8) {
			*NumberOfObject = 3;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 81, 97, 700, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 80, 98, 300, 600, 8 * trans, -13 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			*A = function_tempt;
			*budget = 135;
			*height = 300;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 6;
			*amp = -0.1;
			*frequency = 0.05;
		}
	}
	else if (stage == 4) {
		if (step == 1) {
			*NumberOfObject = 4;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 470, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 470, 440, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 470, 320, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 30;
			*height = 300;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 0;
			*amp = 1.0;
			*frequency = 1.0;

			*explo = false;
			*bomb_population = 1;
			function_bomb_tempt=(Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 590.0, 590.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 2) {
			*NumberOfObject = 8;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 400, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 500, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 600, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 440, 480, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 5, 560, 480, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 81, 97, 450, 430, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[7], 81, 97, 550, 430, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 30;
			*height = 300;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 0;
			*amp = 1.0;
			*frequency = 1.0;

			*explo = false;
			*bomb_population = 1;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 450.0, 560.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 3) {
			*NumberOfObject = 8;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 400, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 600, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 400, 440, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 600, 440, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 5, 400, 320, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[6], 3, 5, 600, 320, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[7], 3, 5, 500, 320, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 30;
			*height = 270;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 0;
			*amp = 1.0;
			*frequency = 1.0;

			*explo = false;
			*bomb_population = 2;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 500.0, 560.0, 50.0);
			set_bomb(&function_bomb_tempt[1], 500.0, 440.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 4) {
			*NumberOfObject = 8;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 400, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 600, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 400, 440, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 600, 440, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[5], 1, 5, 500, 280, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[6], 1, 5, 700, 430, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[7], 1, 5, 300, 430, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 35;
			*height = 270;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 1;
			*amp = 1.0;
			*frequency = 1.0;

			*explo = false;
			*bomb_population = 3;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 500.0, 350.0, 50.0);
			set_bomb(&function_bomb_tempt[1], 700.0, 500.0, 50.0);
			set_bomb(&function_bomb_tempt[2], 300.0, 500.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 5) {
			*NumberOfObject = 8;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 80, 98, 700, 500, -10 * trans, -10 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 430, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 390, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 2, 6, 410, 480, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[5], 80, 98, 300, 50, 0, 0, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[6], 80, 98, 500, 50, 0, 0, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[7], 80, 98, 700, 50, 0, 0, 0, 0, 0.5, -1, 0, 0, 0);
			*A = function_tempt;
			*budget = 50;
			*height = 230;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 3;
			*amp = 0.5;
			*frequency = 2.0;

			*explo = false;
			*bomb_population = 3;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 300.0, 500.0, 50.0);
			set_bomb(&function_bomb_tempt[1], 500.0, 350.0, 50.0);
			set_bomb(&function_bomb_tempt[2], 700.0, 200.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 6) {
			*NumberOfObject = 5;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 400, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 400, 440, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 80, 98, 750, 50, -10 * trans, 10 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[4], 1, 5, 300, 40, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 50;
			*height = 230;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 5;
			*amp = -0.5;
			*frequency = 2.0;

			*explo = false;
			*bomb_population = 3;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 300.0, 100.0, 50.0);
			set_bomb(&function_bomb_tempt[1], 300.0, 300.0, 50.0);
			set_bomb(&function_bomb_tempt[2], 300.0, 500.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 7) {
			*NumberOfObject = 5;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 500, 340, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 580, 400, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 420, 400, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 690, 605, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 40;
			*height = 230;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 0;
			*amp = -0.5;
			*frequency = 2.0;

			*explo = false;
			*bomb_population = 1;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 500.0, 450.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 8) {
			*NumberOfObject = 5;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 98, 99, 380, 707, 0, 0, 0, 0, 0.0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 98, 99, 580, 707, 0, 0, 0, 0, 0.0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 80, 98, 480, 50, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 80, 98, 380, 50, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 80, 98, 580, 50, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 120;
			*height = 250;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 6;
			*amp = 0.8;
			*frequency = 2.0;

			*explo = false;
			*bomb_population = 3;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 480.0, 450.0, 50.0);
			set_bomb(&function_bomb_tempt[1], 380.0, 550.0, 50.0);
			set_bomb(&function_bomb_tempt[2], 580.0, 550.0, 50.0);
			*bomb = function_bomb_tempt;
		}
	}
	else if (stage == 5) {
		if (step == 1) {
			*NumberOfObject = 7;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 640, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 370, 600, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 370, 400, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 370, 520, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[5], 80, 98, 150, 600, 10 * trans, -10 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[6], 1, 5, 500, 420, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 45;
			*height = 230;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 3;
			*amp = 0.5;
			*frequency = 2.0;

			*explo = false;
			*bomb_population = 1;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 500.0, 470.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 2) {
			*NumberOfObject = 17;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 1, 5, 370, 605, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 1, 5, 410, 605, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[3], 1, 5, 450, 605, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[4], 1, 5, 490, 605, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[5], 1, 5, 530, 605, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 1, 5, 390, 565, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[7], 1, 5, 430, 565, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[8], 1, 5, 470, 565, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[9], 1, 5, 510, 565, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[10], 1, 5, 410, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[11], 1, 5, 450, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[12], 1, 5, 490, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[13], 1, 5, 430, 485, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[14], 1, 5, 470, 485, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[15], 1, 5, 450, 445, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[16], 80, 98, 550, 100, -5 * trans, 10 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			*A = function_tempt;
			*budget = 5;
			*height = 470;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 3;
			*amp = 0.5;
			*frequency = 2.0;

			*explo = false;
			*bomb_population = 1;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 550.0, 510.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 3) {
			*NumberOfObject = 13;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 400, 605, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 480, 565, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 320, 565, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 81, 97, 400, 555, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 5, 650, 605, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 3, 5, 650, 525, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[7], 3, 5, 650, 445, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[8], 3, 5, 650, 365, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[9], 3, 5, 650, 285, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[10], 81, 97, 660, 235, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[11], 81, 97, 605, 555, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[12], 80, 98, 150, 50, 10 * trans, 10 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			*A = function_tempt;
			*budget = 90;
			*height = 200;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 1;
			*amp = 1.0;
			*frequency = 2.0;
		}
		else if (step == 4) {
			*NumberOfObject = 3;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 500, 530, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 81, 97, 500, 480, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 55;
			*height = 200;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 1;
			*amp = 1.0;
			*frequency = 2.0;

			*explo = false;
			*bomb_population = 3;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 500.0, 590.0, 50.0);
			set_bomb(&function_bomb_tempt[1], 550.0, 590.0, 50.0);
			set_bomb(&function_bomb_tempt[2], 450.0, 590.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 5) {
			*NumberOfObject = 14;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 4, 640, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 4, 600, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 2, 4, 610, 480, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 4, 620, 400, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 4, 580, 400, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[6], 2, 4, 590, 320, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[7], 3, 4, 600, 240, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[8], 3, 4, 560, 240, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[9], 2, 4, 570, 160, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[10], 81, 97, 545, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[11], 1, 5, 500, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[12], 1, 6, 450, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[13], 81, 97, 450, 595, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 150;
			*height = 200;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 1;
			*amp = 2.0;
			*frequency = 2.0;
		}
		else if (step == 6) {
			*NumberOfObject = 4;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 98, 99, 500, 707, 0, 0, 0, 0, -pi / 18.0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 80, 98, 570, 50, 5 * trans, 10 * trans, 0, 0, 1.0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 80, 98, 470, 100, 5 * trans, 10 * trans, 0, 0, 1.0, 0, 0, 0, 0);
			set_object(&function_tempt[3], 80, 98, 690, 100, 5 * trans, 10 * trans, 0, 0, 1.0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 60;
			*height = 250;
			*starttime = 1;
			*timeduration = 5;
			*endtime = 6;
			*earthquake_type = 6;
			*amp = 0.1;
			*frequency = 1.0;

			*explo = false;
			*bomb_population = 3;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 590.0, 630.0, 50.0);
			set_bomb(&function_bomb_tempt[1], 590.0, 530.0, 50.0);
			set_bomb(&function_bomb_tempt[2], 590.0, 430.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 7) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 98, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 100;
			*height = 570;
			*starttime = 1;
			*timeduration = 5;
			*endtime = 6;
			*earthquake_type = 7;
			*amp = -1.5;
			*frequency = 0.5;
		}
		else if (step == 8) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 98, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 200;
			*height = 550;
			*starttime = 1;
			*timeduration = 5;
			*endtime = 6;
			*earthquake_type = 8;
			*amp = 5.0;
			*frequency = 0.5;
		}
	}
	else if (stage == 6) {
		if (step == 1) {
			*NumberOfObject = 11;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[1], 97, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 96, 99, 536 + 86.22724, 727 + 0.51496, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[0], 95, 99, 536 - 76.56916, 727 - 3.5, 0, 0, 0, 0, 0, 0, 0, 0, 0); // 0.411712
			set_object(&function_tempt[3], 80, 98, 450, 150, 3 * trans, 10 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 536, 500, 0, 0, 0, 0, pi/2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 5, 616, 500, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[6], 80, 98, 280, 270, 0, 0, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[7], 80, 98, 330, 170, 0, 0, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[8], 80, 98, 550, 300, 3 * trans, 10 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[9], 80, 98, 680, 300, 3 * trans, 10 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[10], 80, 98, 810, 300, 3 * trans, 10 * trans, 0, 0, 0.5, -1, 0, 0, 0);
			*A = function_tempt;
			*budget = 45;
			*height = 250;
			*starttime = 1;
			*timeduration = 2;
			*endtime = 3;
			*earthquake_type = 0;
			*amp = 1;
			*frequency = 0.5;

			*explo = false;
			*bomb_population = 2;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 450.0, 520.0, 50.0);
			set_bomb(&function_bomb_tempt[1], 450.0, 420.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 2) {
			*NumberOfObject = 11;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[1], 97, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 96, 99, 536 + 86.22724, 727 + 0.51496, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[0], 95, 99, 536 - 76.56916, 727 - 3.5, 0, 0, 0, 0, 0, 0, 0, 0, 0); // 0.411712
			set_object(&function_tempt[3], 80, 98, 850, 150, 0, 0, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[4], 80, 98, 150, 150, 0, 0, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[5], 1, 5, 171, 500, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 1, 5, 819, 400, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[7], 1, 5, 171, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[8], 80, 98, 950, 230, 0, 0, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[9], 80, 98, 80, 220, 0, 0, 0, 0, 0.5, -1, 0, 0, 0);
			set_object(&function_tempt[10], 80, 98, 80, 420, 0, 0, 0, 0, 0.5, -1, 0, 0, 0);
			*A = function_tempt;
			*budget = 45;
			*height = 250;
			*starttime = 1;
			*timeduration = 3;
			*endtime = 4;
			*earthquake_type = 0;
			*amp = 1;
			*frequency = 0.5;

			*explo = false;
			*bomb_population = 3;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 150.0, 500.0, 50.0);
			set_bomb(&function_bomb_tempt[1], 850.0, 400.0, 50.0);
			set_bomb(&function_bomb_tempt[2], 150.0, 300.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 3) {
			*NumberOfObject = 16;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 400, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 500, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 600, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 440, 480, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 5, 560, 480, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 3, 5, 400, 400, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[7], 3, 5, 500, 400, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[8], 3, 5, 600, 400, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[9], 3, 5, 440, 320, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[10], 3, 5, 560, 320, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[11], 3, 5, 400, 240, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[12], 3, 5, 500, 240, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[13], 3, 5, 600, 240, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[14], 3, 5, 440, 160, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[15], 3, 5, 560, 160, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//			set_object(&function_tempt[6], 81, 97, 450, 430, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//			set_object(&function_tempt[7], 81, 97, 550, 430, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*A = function_tempt;
			*budget = 15;
			*height = 100;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 0;
			*amp = 1.0;
			*frequency = 1.0;

			*explo = false;
			*bomb_population = 3;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 450.0, 560.0, 50.0);
			set_bomb(&function_bomb_tempt[1], 450.0, 400.0, 50.0);
			set_bomb(&function_bomb_tempt[2], 450.0, 240.0, 50.0);
			*bomb = function_bomb_tempt;
		}
		else if (step == 4) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			/*
			set_object(&function_tempt[1], 3, 5, 380, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 500, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 380, 440, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 500, 440, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 5, 500, 360, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 3, 5, 380, 360, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*/
			*A = function_tempt;
			*budget = 373;
			*height = 100;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 7;
			*amp = 0.5;
			*frequency = 0.8;

			/*
			*explo = false;
			*bomb_population = 1;
			function_bomb_tempt = (Bomb *)malloc(sizeof(Bomb)*(*bomb_population));
			set_bomb(&function_bomb_tempt[0], 450.0, 560.0, 50.0);
			*bomb = function_bomb_tempt;
			*/
		}
		else if (step == 5) {
			*NumberOfObject = 11;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 81, 97, 310, 472, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 310, 565, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 81, 97, 430, 352, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 430, 445, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 5, 430, 565, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[6], 81, 97, 550, 232, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[7], 3, 5, 550, 325, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[8], 3, 5, 550, 445, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[9], 3, 5, 550, 565, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[10], 80, 98, 750, 465, -15 * trans, 0, 0, 0, 0, 0, 0, 0, 0);
			/*
			set_object(&function_tempt[1], 3, 5, 380, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 500, 560, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 380, 440, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 500, 440, 0, 0, 0, 0, pi / 2.0f, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 5, 500, 360, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 3, 5, 380, 360, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			*/
			*A = function_tempt;
			*budget = 50;
			*height = 100;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 0;
			*amp = 5;
			*frequency = 0.5;
		}
		else if (step == 6) {
			*NumberOfObject = 14;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[1], 3, 5, 310, 605, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[2], 3, 5, 310, 565, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[3], 3, 5, 360, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[4], 3, 5, 360, 485, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[5], 3, 5, 310, 445, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[6], 3, 5, 310, 405, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[7], 3, 5, 360, 365, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[8], 3, 5, 360, 325, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[9], 81, 97, 395, 595, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[10], 81, 97, 270, 515, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[11], 81, 97, 395, 435, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[12], 81, 97, 270, 355, 0, 0, 0, 0, 0, 0, 0, 0, 0);
			set_object(&function_tempt[13], 80, 98, 750, 365, -15 * trans, 0, 0, 0, 0, 0, 0, 0, 0);

			*A = function_tempt;
			*budget = 40; // minimum 30, easy way 46
			*height = 250;
			*starttime = 1;
			*timeduration = 7;
			*endtime = 8;
			*earthquake_type = 1;
			*amp = 2.0;
			*frequency = 1.0;
		}
		else if (step == 7) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);

			*A = function_tempt;
			*budget = 385; //min 240
			*height = 100;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 3;
			*amp = 3.0;
			*frequency = 5.0;
		}
		else if (step == 8) {
			*NumberOfObject = 1;
			function_tempt = (Object*)malloc(sizeof(Object)*(*NumberOfObject));
			set_object(&function_tempt[0], 99, 99, 500, 707, 0, 0, 0, 0, 0, 0, 0, 0, 0);

			*A = function_tempt;
			*budget = 103;
			*height = 100;
			*starttime = 1;
			*timeduration = 6;
			*endtime = 7;
			*earthquake_type = 1;
			*amp = 6.0;
			*frequency = 5.0;
		}
	}
}

float clamp(float a, float min, float max) {
	return maximum(min, minimum(a, max));
}

void bomb_function(Object A[], int NumberOfObject, Bomb b[], int NumberOfBomb) {
	int i, j, k, l;
	float impulse[2];
	float impulse_point[2];
	for (i = 0; i < NumberOfBomb; i++) {
		for (l = 1; l <= bomb_lenght_divide_num; l++) {
			for (j = 0; j < bomb_angle_divide; j++) {
				if (b[i].explosion[j] == false) {
					for (k = 0; k < NumberOfObject; k++) if((b[i].x[0]-A[k].x[0])* (b[i].x[0] - A[k].x[0]) + (b[i].x[1] - A[k].x[1])* (b[i].x[1] - A[k].x[1])>A[k].shape->max_distance*A[k].shape->max_distance || click_perception(b[i].x[0],b[i].x[1],&A[k])>=0){
						if (click_perception(b[i].x[0] + l*bomb_length_divide*cos((j*2.0*pi) / bomb_angle_divide), b[i].x[1] + l*bomb_length_divide*sin((j*2.0*pi) / bomb_angle_divide), &A[k]) < 0) {
							b[i].explosion[j] = true;
							impulse[0] = b[i].momentum*cos((j*2.0*pi) / bomb_angle_divide);
							impulse[1] = b[i].momentum*sin((j*2.0*pi) / bomb_angle_divide);
							impulse_point[0] = l*bomb_length_divide*cos((j*2.0*pi) / bomb_angle_divide) - (A[k].x[0] - b[i].x[0]);
							impulse_point[1] = l*bomb_length_divide*sin((j*2.0*pi) / bomb_angle_divide) - (A[k].x[1] - b[i].x[1]);
							A[k].v[0] = A[k].v[0] + impulse[0] * A[k].inv_mass;
							A[k].v[1] = A[k].v[1] + impulse[1] * A[k].inv_mass;
							A[k].w = A[k].w + A[k].inv_inertia*v_cross(impulse_point, impulse);
						}
					}
				}
			}
		}
	}
}

void cat_hurt_check(Object object[], int population, cinf contact[], bool * cat_hurt, float cat_hurt_coordinate[]) {
	int i;

	for (i = 0; i < 4 * population; i++) if (contact[i].run == true && (object[contact[i].numA].material.number == 97 || object[contact[i].numB].material.number == 97)) { //if (contact[i].tangent_impulse > 2300*g*0.00005)
		if ((object[contact[i].numA].material.number == 97 && (contact[i].indexA == 0 || contact[i].indexA == 1 || contact[i].indexA == 2)) || (object[contact[i].numB].material.number == 97 && (contact[i].indexB == 0 || contact[i].indexB == 1 || contact[i].indexB == 2 || contact[i].indexB == 3))) {
			if (contact[i].tangent_impulse > 2300 * g*0.0001) { // 기존 0.0001
				*cat_hurt = true;
				cat_hurt_coordinate[0] = object[contact[i].numB].x[0] + contact[i].rB[0];
				cat_hurt_coordinate[1] = object[contact[i].numB].x[1] + contact[i].rB[1];
			}
		}
		else {
			if (contact[i].tangent_impulse > 2300 * g*0.00003) { // 기존 0.00003
				*cat_hurt = true;
				cat_hurt_coordinate[0] = object[contact[i].numB].x[0] + contact[i].rB[0];
				cat_hurt_coordinate[1] = object[contact[i].numB].x[1] + contact[i].rB[1];
			}
		}
	}
}

void print_digit(int budget, int * hundreds, int * tenth, int * units) {
	if (budget >= 100) {
		*hundreds = (budget - (budget % 100)) / 100;
		budget = budget - *hundreds * 100;
	}
	else *hundreds = 0;

	if (budget >= 10) {
		*tenth = (budget - (budget % 10)) / 10;
		budget = budget - *tenth * 10;
	}
	else *tenth = 0;

	*units = budget;
}