#ifndef OPENSTEER_PACK_H
#define OPENSTEER_PACK_H

namespace OpenSteer {

    template<typename SingleType> class Pack
    {
        public:
            Pack():_size(0){}
            int size(void){return _size;}
            void setSize(int size){ _size = size; }

            virtual void push(SingleType e) = 0;
        
        private:
            int _size;
            int mask[PACKSIZE];
    };

}

#endif