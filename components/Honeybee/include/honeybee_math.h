

namespace honeybee_math {
    class Vector2 {
        public:
            void set_x(float x);
            void set_y(float y);
            float get_x();
            float get_y();
            float dot(Vector2 other);
            Vector2 normal();
        private:
            float x, y;
    };

    float map(float value, float in_min, float in_max, float out_min, float out_max);
    float clamp(float value, float min, float max);
    float normalize(float value, float min, float max);
}